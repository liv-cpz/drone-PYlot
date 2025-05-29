#!/usr/bin/env python3
import numpy as np

class BlueROV2Parameters:
    """Physical parameters for BlueROV2 in salt water"""
    
    def __init__(self):
        # Mass and inertia properties (approximate)
        self.mass = 11.5  # kg
        self.weight = self.mass * 9.81  # N
        self.buoyancy = self.weight  # Neutral buoyancy
        self.Ixx = 0.16   # kg·m²
        self.Iyy = 0.16
        self.Izz = 0.16
        
        # Center of gravity (relative to center of buoyancy)
        self.rg = np.array([0, 0, 0.05])  # m
        
        # Added mass coefficients (approximate)
        self.X_udot = -5.0
        self.Y_vdot = -12.0
        self.Z_wdot = -14.0
        self.K_pdot = -0.1
        self.M_qdot = -0.1
        self.N_rdot = -0.1
        
        # Linear damping coefficients
        self.X_u = -4.0
        self.Y_v = -6.0
        self.Z_w = -7.0
        self.K_p = -0.5
        self.M_q = -0.5
        self.N_r = -0.5
        
        # Quadratic damping coefficients
        self.X_uu = -18.0
        self.Y_vv = -21.0
        self.Z_ww = -36.0
        self.K_pp = -1.0
        self.M_qq = -1.0
        self.N_rr = -1.0
        
        # Thruster configuration (simplified for 2D)
        self.B = np.array([
            [1, 1, 0, 0],    # Surge (X)
            [0, 0, 1, 1],    # Sway (Y)
            [0, 0, 0, 0],    # Heave (Z)
            [0, 0, 0, 0],    # Roll (K)
            [0, 0, 0, 0],    # Pitch (M)
            [-0.2, 0.2, -0.2, 0.2]  # Yaw (N)
        ])
        
        # Thruster limits (Newtons)
        self.thruster_max = 50.0
        self.thruster_min = -50.0