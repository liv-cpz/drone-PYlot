"""_summary_
This module implements a PID controller for drone navigation.
Inputs to controller - drone current pose, target pose, and navigation stage.
Outputs from controller - drone velocity commands in x, y, z, and yaw.
"""
from enum import Enum
from simple_pid import PID
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
import logging

import numpy as np
import math

logger = logging.getLogger(__name__)

class NavigationStage(Enum):
    # possibly unnecessary ???
    TAKEOFF = 0
    ALIGNMENT = 1
    APPROACH = 2
    LANDING = 3


@dataclass
class PIDConfig:
    """@dataclass for PID configuration parameters.
    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (float): Desired setpoint for the PID controller.
        output_limits (Tuple[float, float]): Limits for the output of the PID controller.
        sample_time (Optional[float]): Time interval for PID updates, if None, uses default.
    """
    kp: float
    ki: float
    kd: float
    setpoint: float
    output_limits: Tuple[float, float] = (-100, 100)
    sample_time: Optional[float] = None
    
    

class DronePIDController:
    """DronePIDController implements a PID controller for drone navigation.
    It uses PID controllers for x, y, z, and yaw axes to compute velocity commands.   
    It takes a configuration dictionary to initialize the PID controllers.
    Attributes:
        pid_x (PID): PID controller for x-axis velocity.   
        pid_y (PID): PID controller for y-axis velocity.
        pid_z (PID): PID controller for z-axis velocity.
        pid_yaw (PID): PID controller for yaw velocity.
    """  
    pid_x: PID
    pid_y: PID
    pid_z: PID
    pid_yaw: PID

    def __init__(self, config: Dict[str, PIDConfig]):
        self.pid_x = PID(config['x'].kp, config['x'].ki, config['x'].kd,
                         setpoint=config['x'].setpoint,
                         output_limits=config['x'].output_limits,
                         sample_time=config['x'].sample_time)
        self.pid_y = PID(config['y'].kp, config['y'].ki, config['y'].kd,
                         setpoint=config['y'].setpoint,
                         output_limits=config['y'].output_limits,
                         sample_time=config['y'].sample_time)
        self.pid_z = PID(config['z'].kp, config['z'].ki, config['z'].kd,
                         setpoint=config['z'].setpoint,
                         output_limits=config['z'].output_limits,
                         sample_time=config['z'].sample_time)
        self.pid_yaw = PID(config['yaw'].kp, config['yaw'].ki, config['yaw'].kd,
                           setpoint=config['yaw'].setpoint,
                           output_limits=config['yaw'].output_limits,
                           sample_time=config['yaw'].sample_time)
        
        
        self.navigation_state = NavigationStage.TAKEOFF
        
        logger.info("Drone PID controller initialised")
        
        
    def update(self, current_pose: Tuple[float, float, float, float], target_pose: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """
        Update the PID controllers with the current and target poses.
        Args:
            current_pose (Tuple[float, float, float, float]): Current pose of the drone (x, y, z, yaw).
            target_pose (Tuple[float, float, float, float]): Target pose for the drone (x, y, z, yaw).
        Returns:
            Tuple[float, float, float, float]: Velocity commands for x, y, z, and yaw.
        """
        if not current_pose:
            return 0.0, 0.0, 0.0, 0.0
        
        x_error = target_pose[0] - current_pose[0]
        y_error = target_pose[1] - current_pose[1]
        z_error = target_pose[2] - current_pose[2]
        yaw_error = target_pose[3] - current_pose[3]
        
        vx = self.pid_x(x_error)
        vy = self.pid_y(y_error)
        vz = self.pid_z(z_error)
        vyaw = self.pid_yaw(yaw_error)
        
        
        logger.info(f"PID update: vx={vx}, vy={vy}, vz={vz}, vyaw={vyaw}, "
                    f"x_error={x_error}, y_error={y_error}, z_error={z_error}, yaw_error={yaw_error}")
        
        return vx, vy, vz, vyaw
    
    
    def reset_pid(self):
        """Reset PID controllers to initial state.
        """
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        
    
    
"""
1. Marker based pose estimation:
    Current pose: (x, y, z, yaw)
    Target pose: (x_target, y_target, z_target, yaw_target)
2. Pure Pursuit:
    initial pose: (x, y, z, yaw)
    calculate theta = arctan(x_target/y_target)
    euclidean distance = sqrt((x_target - x)^2 + (y_target - y)^2)

...
"""