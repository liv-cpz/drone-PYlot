#!/usr/bin/env python3
import numpy as np
from ..dynamics.marine_dynamics import MarineDynamics
from ..control.pid_controller import PIDController
from ..environment.hull_profiles import HullProfiles

class TerminalSimulator:
    def __init__(self):
        self.hull_profiles = HullProfiles()
        self.hull_points = self.hull_profiles.tanker_hull(length=10.0)
        
    def simulate_controller(self, pid, color, label):
        """Simulate a single controller until x=10m is reached"""
        dynamics = MarineDynamics()
        trajectory = []
        target_idx = 0
        dt = 0.1
        max_sim_time = 120.0  # Maximum simulation time (seconds)
        sim_time = 0
        
        while sim_time < max_sim_time:
            current_x = dynamics.eta[0]
            if current_x >= 10.0:  # Stop when we reach the end of the hull
                break
                
            # Get current target point
            target_point = np.array([
                self.hull_points[target_idx, 0], 
                self.hull_points[target_idx, 1], 
                0, 0, 0, 0
            ])
            
            # Move to next target point if close enough
            if np.linalg.norm(dynamics.eta[:2] - target_point[:2]) < 0.2:
                target_idx = min(target_idx + 1, len(self.hull_points) - 1)
            
            # Compute control forces
            control = pid.compute(target_point, dynamics.eta)
            
            # Apply control (only in x and y for 2D tracking)
            tau = np.zeros(6)
            tau[:2] = control[:2]
            
            # Update dynamics
            dynamics.update(tau, dt)
            
            # Store position for plotting
            trajectory.append(dynamics.eta[:2].copy())
            sim_time += dt
        
        return np.array(trajectory), color, label

    def get_controller_params(self, index):
        """Prompt user for controller parameters"""
        print(f"\nController {index + 1} parameters:")
        kp = float(input("Enter proportional gain (Kp): "))
        ki = float(input("Enter integral gain (Ki): "))
        kd = float(input("Enter derivative gain (Kd): "))
        
        pid = PIDController()
        pid.set_gains(
            [kp, kp, 1.0, 0.5, 0.5, 1.0],
            [ki, ki, 0.05, 0.01, 0.01, 0.05],
            [kd, kd, 0.5, 0.1, 0.1, 0.5]
        )
        
        color = input("Enter plot color (e.g., 'r' for red, 'b' for blue): ")
        label = input("Enter controller label: ")
        
        return pid, color, label

    def run_comparison(self, num_controllers):
        """Run comparison of multiple controllers"""
        controllers = []
        for i in range(num_controllers):
            pid, color, label = self.get_controller_params(i)
            controllers.append((pid, color, label))
        
        print("\nRunning simulations...")
        results = []
        for pid, color, label in controllers:
            print(f"Simulating {label}...")
            trajectory, color, label = self.simulate_controller(pid, color, label)
            results.append((trajectory, color, label))
        
        return results, self.hull_points