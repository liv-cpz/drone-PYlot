#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from enum import Enum, auto
from datetime import datetime
from std_msgs.msg import Empty
from sensor_msgs.msg import BatteryState, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # For bbox input

from aruco_trajectory_estimator import ArucoTrajectoryEstimator

class DroneState(Enum):
    START = auto()
    LAUNCH = auto()
    HOVER = auto()
    DETECT = auto()
    ADJUST = auto()
    SEARCH = auto()
    FOLLOW = auto()
    TRACK = auto()
    SEARCH_PATTERN = auto()
    LANDING_INIT = auto()
    LANDING_FINAL = auto()
    STOP = auto()
    EMERGENCY_LAND = auto()

class TelloFSM(Node):
    def __init__(self):
        super().__init__('tello_fsm')
        self.current_state = DroneState.START
        self.log_state_transition(None, self.current_state)
        self.get_logger().info(f"Initial state: {self.current_state.name}")

        # Drone status
        self.battery_percentage = 100.0
        self.depth_readings = []
        self.depth_duration = Duration(seconds=2.0)
        self.depth_noise_threshold = 0.01  # 1 cm

        # Flight command publishers
        self.takeoff_pub = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/tello/land', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)

        # Sensor subscribers
        self.create_subscription(BatteryState, '/battery_status', self.battery_callback, 10)
        self.create_subscription(Range, '/range/downward', self.depth_callback, 10)

        # Subscribe to ArUco bounding box published as Float32MultiArray: [x_min, y_min, x_max, y_max]
        self.create_subscription(Float32MultiArray, '/aruco_bbox', self.aruco_callback, 10) #REPLACE WITH THE ACTUAOL SUBSCRIBER FROM FLORA

        # ArUco tracking
        self.trajectory_estimator = ArucoTrajectoryEstimator(window_size=30)
        self.object_visible = False
        self.last_seen_time = self.get_clock().now()

        # State tracking
        self.timer = self.create_timer(1.0 / 30.0, self.run_state_machine)  # Run at 30Hz matching camera
        self.state_start_time = self.get_clock().now()

        # Search pattern control
        self.search_index = 0
        self.search_direction_stage = 0  # 0 = forward, 1 = back
        self.search_start_time = None

    def aruco_callback(self, msg: Float32MultiArray):
        bbox = msg.data
        if len(bbox) == 4:
            self.trajectory_estimator.add_bbox(tuple(bbox))
            self.object_visible = True
            self.last_seen_time = self.get_clock().now()
        else:
            self.object_visible = False

    def run_state_machine(self):
        now = self.get_clock().now()

        if self.check_emergency_condition():
            self.transition_to(DroneState.EMERGENCY_LAND)
            return

        # Object detection logic simplified: just use aruco visibility
        if self.object_visible:
            self.last_seen_time = now
            if self.current_state in [DroneState.TRACK, DroneState.SEARCH_PATTERN]:
                self.transition_to(DroneState.FOLLOW)
                return

        if self.current_state == DroneState.START:
            self.transition_to(DroneState.LAUNCH)

        elif self.current_state == DroneState.LAUNCH:
            self.command_takeoff()
            self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.HOVER:
            self.transition_to(DroneState.DETECT)

        elif self.current_state == DroneState.DETECT:
            if self.object_visible:
                self.transition_to(DroneState.ADJUST)
            else:
                self.transition_to(DroneState.SEARCH)

        elif self.current_state == DroneState.ADJUST:
            self.transition_to(DroneState.FOLLOW)

        elif self.current_state == DroneState.SEARCH:
            self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.FOLLOW:
            # If lost sight for more than 5 seconds, go to TRACK state
            if (now - self.last_seen_time).nanoseconds >= 5e9:
                self.transition_to(DroneState.TRACK)

        elif self.current_state == DroneState.TRACK:
            # Use smoothed trajectory to generate velocity commands toward ArUco marker
            if (now - self.last_seen_time).nanoseconds >= 5e9:
                self.transition_to(DroneState.SEARCH_PATTERN)
            else:
                self.track_using_trajectory()

        elif self.current_state == DroneState.SEARCH_PATTERN:
            if self.search_index >= 4:
                self.get_logger().warn("Search pattern failed. Initiating emergency landing.")
                self.transition_to(DroneState.EMERGENCY_LAND)
                return

            if self.search_start_time is None:
                self.search_start_time = now
                self.command_move(self.get_search_direction())
                return

            elapsed = (now - self.search_start_time).nanoseconds * 1e-9

            if self.search_direction_stage == 0 and elapsed >= 2.0:
                self.command_move((-self.get_search_direction()[0], -self.get_search_direction()[1]))
                self.search_direction_stage = 1
                self.search_start_time = now

            elif self.search_direction_stage == 1 and elapsed >= 2.0:
                self.search_index += 1
                self.search_direction_stage = 0
                self.search_start_time = None

        elif self.current_state == DroneState.LANDING_INIT:
            self.transition_to(DroneState.LANDING_FINAL)

        elif self.current_state == DroneState.LANDING_FINAL:
            if self.landing_successful():
                self.command_land()
                self.transition_to(DroneState.STOP)
            else:
                self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.EMERGENCY_LAND:
            self.command_emergency_land()
            self.transition_to(DroneState.STOP)

        elif self.current_state == DroneState.STOP:
            self.get_logger().info("FSM in STOP state. Awaiting restart.")

    def get_search_direction(self):
        directions = [
            (0.2, 0.0),  # Forward
            (0.0, -0.2), # Right
            (-0.2, 0.0), # Backward
            (0.0, 0.2)   # Left
        ]
        return directions[self.search_index]

    def track_using_trajectory(self):
        predicted_trajectory = self.trajectory_estimator.predict_trajectory(n_points=30)
        if predicted_trajectory.size == 0:
            self.get_logger().warn("Not enough points to predict trajectory.")
            return

        # Use the first predicted future point as target
        target_x, target_y = predicted_trajectory[0]

        # Proportional controller gains (tune as needed)
        Kp_x = 0.5
        Kp_y = 0.5

        twist = Twist()
        twist.linear.x = Kp_x * target_x
        twist.linear.y = Kp_y * target_y

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Tracking predicted trajectory: vx={twist.linear.x:.3f}, vy={twist.linear.y:.3f}")


    def transition_to(self, new_state):
        if new_state != self.current_state:
            self.log_state_transition(self.current_state, new_state)
            self.get_logger().info(f"Transitioning from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()

            if new_state == DroneState.SEARCH_PATTERN:
                self.search_index = 0
                self.search_direction_stage = 0
                self.search_start_time = None

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage * 100.0

    def depth_callback(self, msg):
        now = self.get_clock().now()
        self.depth_readings.append((now, msg.range))
        self.depth_readings = [(t, r) for (t, r) in self.depth_readings if (now - t) < self.depth_duration]

    def check_emergency_condition(self):
        return self.battery_percentage < 10.0

    def landing_successful(self):
        if len(self.depth_readings) < 5:
            return False
        readings = [r for (_, r) in self.depth_readings]
        min_depth = min(readings)
        max_depth = max(readings)
        return min_depth >= 0.03 and (max_depth - min_depth) <= self.depth_noise_threshold

    def command_takeoff(self):
        self.get_logger().info("Taking off...")
        self.takeoff_pub.publish(Empty())

    def command_land(self):
        self.get_logger().info("Landing...")
        self.land_pub.publish(Empty())

    def command_emergency_land(self):
        self.get_logger().warn("Emergency landing...")
        self.land_pub.publish(Empty())

    def command_move(self, direction):
        twist = Twist()
        twist.linear.x = direction[0]
        twist.linear.y = direction[1]
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Executing search move: vx={direction[0]}, vy={direction[1]}")

def main(args=None):
    rclpy.init(args=args)
    fsm_node = TelloFSM()
    rclpy.spin(fsm_node)
    fsm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
