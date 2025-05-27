
import logging
import djitellopy

from drone_controller import PIDConfig, DronePIDController, NavigationStage


# add VIO for state estimation
# add Target pose info 



logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger(__name__)


class DroneFly:
    
    def __init__(self):
        self.drone = None
        self.drone_controller = None

    def connect_tello(self):
        """Initialize the drone connection."""
        try:
            self.drone = djitellopy.Tello()
            self.drone.connect()
            logger.info("Drone connected successfully.")
        except Exception as e:
            logger.error(f"Failed to connect to drone: {e}")
            raise
        
    def initialise_controller(self):
        """Initialize the drone controller with PID configuration."""
        try:
            #todo tune controller params
            config = {
                'x': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=0.0),
                'y': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=0.0),
                'z': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=1.0),  # Setpoint for takeoff
                'yaw': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=0.0)
            }
            self.drone_controller = DronePIDController(config)
            logger.info("Drone controller initialised successfully.")
        except Exception as e:
            logger.error(f"Failed to initialise drone controller: {e}")
            raise

def main():
    run = DroneFly()
    
    # Example usage:
    try:
        run.connect_tello()
        run.initialise_controller()
        
        # Example usage of the drone controller
        current_pose = (0.0, 0.0, 0.0, 0.0)  # Current pose (x, y, z, yaw)
        target_pose = (1.0, 1.0, 1.0, 90.0)  # Target pose (x, y, z, yaw)
        
        velocity_commands = run.drone_controller.update(current_pose, target_pose)
        logger.info(f"Velocity commands: {velocity_commands}")
        
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        if run.drone:
            run.drone.end()
            logger.info("Drone connection closed.")


if __name__ == "__main__":
    main()