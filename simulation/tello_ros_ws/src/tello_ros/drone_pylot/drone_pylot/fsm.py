#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from enum import Enum, auto
from datetime import datetime
from std_msgs.msg import Empty, Header
from sensor_msgs.msg import BatteryState, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # For bbox input
from tello_msgs.srv import TelloAction  # Service for Tello commands
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


# For trajectory estimation (simplified version)
class ArucoTrajectoryEstimator:
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.bbox_history = []
        self.velocity_history = []
        
    def add_bbox(self, bbox):
        if len(bbox) != 4:
            return
            
        # Calculate center
        x_min, y_min, x_max, y_max = bbox
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        
        # Add to history
        self.bbox_history.append((center_x, center_y))
        if len(self.bbox_history) > self.window_size:
            self.bbox_history.pop(0)
            
        # Calculate velocity if we have enough points
        if len(self.bbox_history) >= 2:
            prev_x, prev_y = self.bbox_history[-2]
            current_x, current_y = self.bbox_history[-1]
            vel_x = current_x - prev_x
            vel_y = current_y - prev_y
            self.velocity_history.append((vel_x, vel_y))
            if len(self.velocity_history) > self.window_size:
                self.velocity_history.pop(0)
    
    def predict_trajectory(self, n_points=10):
        if len(self.bbox_history) < 2:
            return np.array([])
            
        # Simple linear prediction
        current_x, current_y = self.bbox_history[-1]
        
        # Calculate average velocity
        if self.velocity_history:
            avg_vel_x = sum(v[0] for v in self.velocity_history) / len(self.velocity_history)
            avg_vel_y = sum(v[1] for v in self.velocity_history) / len(self.velocity_history)
        else:
            avg_vel_x, avg_vel_y = 0.0, 0.0
            
        # Generate predicted points
        trajectory = []
        for i in range(1, n_points + 1):
            pred_x = current_x + avg_vel_x * i
            pred_y = current_y + avg_vel_y * i
            trajectory.append((pred_x, pred_y))
            
        return np.array(trajectory)

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
        self.target_height = 1.0  # Target height in meters (1m above target)
        self.use_service = False  # Flag to determine if we should use service or topics
        self.marker_visible = False

        self.create_subscription(
            Bool,
            '/aruco_visible',
            self.visible_callback,
            10
        )

        self.aruco_pose = None  # 最新的 ArUco 位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/aruco/pose",
            self.aruco_pose_callback,
            10
        )

        # Create a callback group for services to allow parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # Service client for Tello commands (for real drone)
        self.tello_action_client = self.create_client(
            TelloAction,
            '/drone1/tello_action',         # FL: added /drone1
            callback_group=self.cb_group
        )

        # Topic publishers for simulation
        self.takeoff_pub = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/tello/land', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sensor subscribers
        self.create_subscription(BatteryState, '/battery_status', self.battery_callback, 10)
        self.create_subscription(Range, '/range/downward', self.depth_callback, 10)

        # Subscribe to ArUco bounding box
        self.create_subscription(Float32MultiArray, '/aruco_bbox', self.aruco_callback, 10)

        # ArUco tracking
        self.trajectory_estimator = ArucoTrajectoryEstimator(window_size=30)
        self.object_visible = False
        self.last_seen_time = self.get_clock().now()
        self.bbox_center_x = 0.0
        self.bbox_center_y = 0.0
        self.bbox_width = 0.0
        self.bbox_height = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # State tracking
        # self.timer = self.create_timer(1.0 / 30.0, lambda: asyncio.create_task(self.async_state_machine()))
        self.state_start_time = self.get_clock().now()

        # Search pattern control
        self.search_index = 0
        self.search_direction_stage = 0  # 0 = forward, 1 = back
        self.search_start_time = None

        # Check if we should use service or topics
        self.determine_communication_method()

    def visible_callback(self, msg):
        self.marker_visible = msg.data
        # self.get_logger().info(f"[aruco_visible] Updated: {self.marker_visible}")

    def aruco_pose_callback(self, msg: PoseStamped):
        self.aruco_pose = msg.pose            

    def determine_communication_method(self):
        """Check if the Tello service is available, otherwise use topics"""
        if self.tello_action_client.wait_for_service(timeout_sec=1.0):
            self.use_service = True
            self.get_logger().info("Using Tello service for commands")
        else:
            self.use_service = False
            self.get_logger().info("Using standard topics for commands (simulation mode)")

    async def send_tello_command(self, command):
        """Send a command to the Tello drone via service or topic"""
        if self.use_service:
            req = TelloAction.Request()
            req.cmd = command

            self.get_logger().info(f"[send_tello_command] Calling service with: {command}")
            future = self.tello_action_client.call_async(req)

            # ✅ 使用 asyncio 等待 future 完成
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                await asyncio.sleep(0.1)

            if future.done():
                response = future.result()
                self.get_logger().info(f"Tello command '{command}' response: {response.rc}")
                return response.rc
            else:
                self.get_logger().error("Future never completed!")
                return -1
        else:
            # For simulation, we'll use topics
            if command == "takeoff":
                self.takeoff_pub.publish(Empty())
                return 0
            elif command == "land":
                self.land_pub.publish(Empty())
                return 0
            elif command == "emergency":
                self.land_pub.publish(Empty())  # Just land for simulation
                return 0
            elif command.startswith("rc"):
                # Parse RC command and convert to Twist
                parts = command.split()
                if len(parts) >= 4:
                    try:
                        lr = int(parts[1]) / 100.0  # Convert from -100..100 to -1..1
                        fb = int(parts[2]) / 100.0
                        ud = int(parts[3]) / 100.0
                        yaw = int(parts[4]) / 100.0 if len(parts) > 4 else 0.0
                        
                        twist = Twist()
                        twist.linear.x = fb
                        twist.linear.y = lr
                        twist.linear.z = ud
                        twist.angular.z = yaw
                        self.cmd_vel_pub.publish(twist)
                        return 0
                    except (ValueError, IndexError):
                        self.get_logger().error(f"Invalid RC command format: {command}")
                        return -1
            return -1

    async def run_state_machine(self):
        # Wrap the async state machine in an event loop
        # asyncio.create_task(self.async_state_machine())
        await self.async_state_machine()

    async def async_state_machine(self):
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
            if await self.command_takeoff():
                self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.HOVER:
            await self.command_hover()
            self.transition_to(DroneState.DETECT)

        elif self.current_state == DroneState.DETECT:
            if self.object_visible:
                self.transition_to(DroneState.ADJUST)
            else:
                self.transition_to(DroneState.SEARCH)

        elif self.current_state == DroneState.ADJUST:
            await self.adjust_position()
            
            if not self.marker_visible:
                self.get_logger().warn("[ADJUST] ArUco lost! Sending stop command.")
                if self.use_service:
                    await self.send_tello_command("rc 0 0 0 0")
                    self.transition_to(DroneState.SEARCH)
                else:
                    stop_twist = Twist()
                    stop_twist.linear.x = 0.0
                    stop_twist.linear.y = 0.0
                    stop_twist.linear.z = 0.0
                    stop_twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(stop_twist)
                    self.transition_to(DroneState.SEARCH)
                return

            if self.is_centered():
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
                await self.command_move(self.get_search_direction())
                return

            elapsed = (now - self.search_start_time).nanoseconds * 1e-9

            if self.search_direction_stage == 0 and elapsed >= 2.0:
                await self.command_move((-self.get_search_direction()[0], -self.get_search_direction()[1]))
                self.search_direction_stage = 1
                self.search_start_time = now

            elif self.search_direction_stage == 1 and elapsed >= 2.0:
                self.search_index += 1
                self.search_direction_stage = 0
                self.search_start_time = None

        elif self.current_state == DroneState.LANDING_INIT:
            self.transition_to(DroneState.LANDING_FINAL)

        elif self.current_state == DroneState.LANDING_FINAL:
            # if self.landing_successful():
            #     await self.command_land()
            #     self.transition_to(DroneState.STOP)
            # else:
            # self.transition_to(DroneState.HOVER)
            await self.command_land()
            self.transition_to(DroneState.STOP)

        elif self.current_state == DroneState.EMERGENCY_LAND:
            await self.command_emergency_land()
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

    # def adjust_position(self):
    #     """Adjust drone position to center the ArUco marker in the image"""
    #     # Target center is (0.5, 0.5) for normalized coordinates
    #     target_x = 0.5
    #     target_y = 0.5
        
    #     # Calculate errors
    #     error_x = self.bbox_center_x - target_x
    #     error_y = self.bbox_center_y - target_y
        
    #     # Proportional control gains (tune these as needed)
    #     Kp_x = 0.5
    #     Kp_y = 0.5
        
    #     # Create Twist message
    #     twist = Twist()
    #     twist.linear.x = -Kp_y * error_y  # Move forward/back to center y position
    #     twist.linear.y = -Kp_x * error_x  # Move left/right to center x position
        
    #     # Add small z adjustment based on bbox size (to maintain distance)
    #     if self.bbox_width > 0 and self.bbox_height > 0:
    #         avg_size = (self.bbox_width + self.bbox_height) / 2.0
    #         size_error = avg_size - 0.3  # Target size is 0.3 of image width/height
    #         twist.linear.z = -0.1 * size_error  # Move up/down to maintain distance
        
    #     self.cmd_vel_pub.publish(twist)
    #     self.get_logger().info(f"Adjusting position: vx={twist.linear.x:.3f}, vy={twist.linear.y:.3f}, vz={twist.linear.z:.3f}")

    async def adjust_position(self):
        """Adjust drone position to center the ArUco marker in the image"""
        target_x = 0.03
        target_y = 0.03
        target_z = 0

        if not self.marker_visible or self.aruco_pose is None:
            self.get_logger().warn("[ADJUST] ArUco lost or pose missing! Sending stop command.")
            await self.send_tello_command("rc 0 0 0 0")
            self.transition_to(DroneState.HOVER)
            return

        # Current Error
        error_x = target_x-self.aruco_pose.position.x #- target_x
        error_y = self.aruco_pose.position.y - target_y
        error_z = target_z-self.aruco_pose.position.z #- target_z

        self.get_logger().info(f"Current error: {error_x} {error_y} {error_z}")

        # P control
        Kp = 0.5
        vx = -Kp * error_x  # 前后 → rc fb
        vy = -Kp * error_y  # 左右 → rc lr
        vz = Kp * error_z   # 上下 → rc ud

        # Limits
        def clamp(v, limit):
            return max(min(v, limit), -limit)

        vx = clamp(vx, 0.3)
        vy = clamp(vy, 0.3)
        vz = clamp(vz, 0.3)

        # Stop check
        if abs(error_x) < 0.01 and abs(error_y) < 0.01:
            self.get_logger().info("[ADJUST] Target is at center. STOPPED.")
            await self.send_tello_command("rc 0 0 0 0")
            self.transition_to(DroneState.LANDING_INIT)
            return

        rc_lr = vx*0.2
        rc_fb = vy*0.2
        rc_ud = 0
        rc_yaw = 0

        self.get_logger().info(f"ADJUST RC: rc {rc_lr:.5f} {rc_fb:.5f} {rc_ud:.5f} {rc_yaw:.5f}")

        if self.use_service:
            self.get_logger().info("Service used, speed command sent")
            await self.send_tello_command(f"rc {rc_lr:.5f} {rc_fb:.5f} {rc_ud:.5f} {rc_yaw:.5f}")
        else:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            self.cmd_vel_pub.publish(twist)

    def is_centered(self):
        """Check if the marker is centered within a threshold"""
        target_x = 0.5
        target_y = 0.5
        threshold = 0.05  # 5% of image width/height 
        
        x_error = abs(self.bbox_center_x - target_x)
        y_error = abs(self.bbox_center_y - target_y)
        
        return x_error < threshold and y_error < threshold

    async def command_hover(self):
        """Keep the drone stationary in the air"""
        if self.use_service:
            # For real drone: send RC command with all zeros except throttle
            await self.send_tello_command("rc 0 0 0 0.05")
        else:
            # For simulation: send zero velocities
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Hovering in place")

    async def command_move(self, direction):
        """Move the drone in a specific direction"""
        if self.use_service:
            # Convert direction to RC commands
            rc_x = int(direction[0] * 100)
            rc_y = int(direction[1] * 100)
            await self.send_tello_command(f"rc {rc_x} {rc_y} 0 1")
        else:
            # For simulation: use Twist messages
            twist = Twist()
            twist.linear.x = direction[0]
            twist.linear.y = direction[1]
            self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Executing move: {direction}")

    # FL: updated
    async def command_takeoff(self):
        self.get_logger().info("Taking off...")
        if self.use_service:
            self.get_logger().info("Service method used: ")
            try:
                response = await self.send_tello_command("takeoff")
                self.get_logger().info(f"Takeoff command response rc={response}")  # 🔍 Debug
                if response in [0, 1]:
                    await self.send_tello_command("rc 0 0 50 0")  # Ascend
                    await asyncio.sleep(2.0)  # wait to ascend
                    await self.send_tello_command("rc 0 0 0 0")  # Stop ascending
                    self.get_logger().info("Takeoff complete")  # 🔍 Debug
                    return True
                else:
                    self.get_logger().warn("Takeoff response not 0")
            except Exception as e:
                self.get_logger().error(f"Takeoff service call failed: {e}")
            return False
        else:
            self.get_logger().info("Else: ")
            self.takeoff_pub.publish(Empty())
            self.get_logger().info("Simulated takeoff published")
            await asyncio.sleep(2.0)
            self.get_logger().info("Simulated takeoff complete")
            return True
    
    # FL: updated
    async def command_land(self):
        self.get_logger().info("Landing...")
        if self.use_service:
            self.get_logger().info("Service method used: ")
            try:
                response = await self.send_tello_command("land")
                self.get_logger().info(f"Land command response rc={response}")  # 🔍 Debug
                if response in [0, 1]:
                    await self.send_tello_command("rc 0 0 -50 0")  # Ascend
                    await asyncio.sleep(2.0)  # wait to ascend
                    await self.send_tello_command("rc 0 0 0 0")  # Stop ascending
                    self.get_logger().info("Land complete")  # 🔍 Debug
                    return True
                else:
                    self.get_logger().warn("TLand response not 0")
            except Exception as e:
                self.get_logger().error(f"Land service call failed: {e}")
            return False
        else:
            self.get_logger().info("Else: ")
            self.takeoff_pub.publish(Empty())
            self.get_logger().info("Simulated takeoff published")
            await asyncio.sleep(2.0)
            self.get_logger().info("Simulated takeoff complete")
            return True

    async def command_land(self):
        """Command the drone to land"""
        self.get_logger().info("Landing...")
        if self.use_service:
            await self.send_tello_command("land")
        else:
            self.land_pub.publish(Empty())

    async def command_emergency_land(self):
        """Command the drone to land immediately"""
        self.get_logger().warn("Emergency landing...")
        if self.use_service:
            await self.send_tello_command("emergency")
        else:
            self.land_pub.publish(Empty())

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
        self.get_logger().info(f"Battery: {self.battery_percentage:.1f}%")

    def depth_callback(self, msg):
        now = self.get_clock().now()
        self.depth_readings.append((now, msg.range))
        self.depth_readings = [(t, r) for (t, r) in self.depth_readings if (now - t) < self.depth_duration]
        self.get_logger().info(f"Depth: {msg.range:.2f}m")

    def aruco_callback(self, msg: Float32MultiArray):
        bbox = msg.data
        if len(bbox) == 4:
            self.trajectory_estimator.add_bbox(tuple(bbox))
            self.object_visible = True
            self.last_seen_time = self.get_clock().now()
            
            # Calculate bbox center and dimensions
            self.bbox_center_x = (bbox[0] + bbox[2]) / 2.0
            self.bbox_center_y = (bbox[1] + bbox[3]) / 2.0
            self.bbox_width = bbox[2] - bbox[0]
            self.bbox_height = bbox[3] - bbox[1]
            
            # self.get_logger().info(f"ArUco detected at ({self.bbox_center_x:.2f}, {self.bbox_center_y:.2f})")
        else:
            self.object_visible = False

    def check_emergency_condition(self):
        return self.battery_percentage < 10.0

    def landing_successful(self):
        if len(self.depth_readings) < 5:
            return False
        readings = [r for (_, r) in self.depth_readings]
        min_depth = min(readings)
        max_depth = max(readings)
        return min_depth >= 0.03 and (max_depth - min_depth) <= self.depth_noise_threshold

    def log_state_transition(self, previous_state, new_state):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if previous_state:
            self.get_logger().info(f"[{timestamp}] State change: {previous_state.name} -> {new_state.name}")
        else:
            self.get_logger().info(f"[{timestamp}] Initial state: {new_state.name}")


async def spin_async(node, executor):
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            await node.async_state_machine()
            await asyncio.sleep(1.0 / 30.0)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    fsm_node = TelloFSM()

    executor = MultiThreadedExecutor()
    executor.add_node(fsm_node)

    try:
        asyncio.run(spin_async(fsm_node, executor)) 
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()