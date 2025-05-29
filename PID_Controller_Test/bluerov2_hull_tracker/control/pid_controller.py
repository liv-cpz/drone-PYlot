#!/usr/bin/env python3
import numpy as np

class PIDController:
    """6-DOF PID controller with anti-windup and rate limiting"""
    
    def __init__(self):
        # PID gains for each DOF [x, y, z, phi, theta, psi]
        self.Kp = np.array([2.0, 2.0, 1.0, 0.5, 0.5, 1.0])
        self.Ki = np.array([0.1, 0.1, 0.05, 0.01, 0.01, 0.05])
        self.Kd = np.array([1.0, 1.0, 0.5, 0.1, 0.1, 0.5])
        
        # Controller state
        self.prev_error = np.zeros(6)
        self.integral = np.zeros(6)
        self.derivative = np.zeros(6)
        self.prev_control = np.zeros(6)
        
        # Limits
        self.integral_limit = 5.0
        self.control_rate_limit = 10.0  # N/s or Nm/s
        self.dt = 0.1  # Default time step
        
    def set_gains(self, Kp, Ki, Kd):
        """Set PID gains for all DOFs"""
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        
    def set_time_step(self, dt):
        """Set controller time step"""
        self.dt = dt
        
    def compute(self, setpoint, current_state):
        """
        Compute control output with anti-windup and rate limiting
        setpoint: desired [x, y, z, phi, theta, psi]
        current_state: current [x, y, z, phi, theta, psi]
        Returns: control vector [X, Y, Z, K, M, N]
        """
        error = setpoint - current_state
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        for i in range(6):
            if abs(self.integral[i]) > self.integral_limit:
                self.integral[i] = np.sign(self.integral[i]) * self.integral_limit
        I = self.Ki * self.integral
        
        # Derivative term (filtered)
        self.derivative = 0.8 * self.derivative + 0.2 * ((error - self.prev_error) / self.dt)
        D = self.Kd * self.derivative
        
        # Compute raw control output
        control = P + I + D
        
        # Apply rate limiting
        control_rate = (control - self.prev_control) / self.dt
        for i in range(6):
            if abs(control_rate[i]) > self.control_rate_limit:
                max_change = self.control_rate_limit * self.dt
                control[i] = self.prev_control[i] + np.sign(control_rate[i]) * max_change
        
        # Update previous values
        self.prev_error = error
        self.prev_control = control
        
        return control
    
    def reset(self):
        """Reset controller state"""
        self.prev_error = np.zeros(6)
        self.integral = np.zeros(6)
        self.derivative = np.zeros(6)
        self.prev_control = np.zeros(6)