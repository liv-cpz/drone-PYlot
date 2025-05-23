#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum, auto
from datetime import datetime

class DroneState(Enum):
    START = auto()
    LAUNCH = auto()
    HOVER = auto()
    DETECT = auto()
    ADJUST = auto()
    SEARCH = auto()
    FOLLOW = auto()
    TRACK = auto()
    LAND = auto()
    STOP = auto()
    EMERGENCY_LAND = auto()

class TelloFSM(Node):
    def __init__(self):
        super().__init__('tello_fsm')
        self.current_state = DroneState.START
        self.log_state_transition(None, self.current_state)
        self.get_logger().info(f"Initial state: {self.current_state.name}")

        # Timer to run FSM at 10Hz
        self.timer = self.create_timer(0.1, self.run_state_machine)

    def run_state_machine(self):
        if self.check_emergency_condition():
            self.transition_to(DroneState.EMERGENCY_LAND)
            return

        if self.current_state == DroneState.START:
            self.transition_to(DroneState.LAUNCH)

        elif self.current_state == DroneState.LAUNCH:
            self.command_takeoff()
            self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.HOVER:
            self.transition_to(DroneState.DETECT)

        elif self.current_state == DroneState.DETECT:
            if self.object_detected():
                self.transition_to(DroneState.ADJUST)
            else:
                self.transition_to(DroneState.SEARCH)

        elif self.current_state == DroneState.ADJUST:
            self.transition_to(DroneState.FOLLOW)

        elif self.current_state == DroneState.SEARCH:
            self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.FOLLOW:
            self.transition_to(DroneState.TRACK)

        elif self.current_state == DroneState.TRACK:
            self.transition_to(DroneState.HOVER)

        elif self.current_state == DroneState.LAND:
            self.command_land()
            self.transition_to(DroneState.STOP)

        elif self.current_state == DroneState.EMERGENCY_LAND:
            self.command_emergency_land()
            self.transition_to(DroneState.STOP)

        elif self.current_state == DroneState.STOP:
            self.get_logger().info("FSM in STOP state. Awaiting node restart to reset.")

    def transition_to(self, new_state):
        if new_state != self.current_state:
            self.log_state_transition(self.current_state, new_state)
            self.get_logger().info(f"Transitioning from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state

    def log_state_transition(self, from_state, to_state):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_line = f"[{timestamp}] {from_state.name if from_state else 'None'} → {to_state.name}\n"
        with open("FSM_log.txt", "a") as log_file:
            log_file.write(log_line)

    def check_emergency_condition(self):
        # Replace with real emergency condition, battery low
        return False

    def command_takeoff(self):
        self.get_logger().info("Drone taking off...")

    def command_land(self):
        self.get_logger().info("Drone landing...")

    def command_emergency_land(self):
        self.get_logger().warn("Emergency landing initiated!")

    def object_detected(self):
        # Placeholder detection logic
        return True


def main(args=None):
    rclpy.init(args=args)
    fsm_node = TelloFSM()
    rclpy.spin(fsm_node)
    fsm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
