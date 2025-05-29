#!/usr/bin/env python3
import numpy as np
from .bluerov_model import BlueROV2Parameters

class MarineDynamics:
    """Implementation of Fossen's equations for marine vehicles"""
    
    def __init__(self):
        self.params = BlueROV2Parameters()
        self.eta = np.zeros(6)  # Position/Euler angles [x,y,z,φ,θ,ψ]
        self.nu = np.zeros(6)   # Body-fixed velocities [u,v,w,p,q,r]
        self.time = 0.0
        
    def update(self, tau, dt):
        phi, theta, psi = self.eta[3], self.eta[4], self.eta[5]
        
        # Rotation matrix for linear velocities
        R = self._rotation_matrix(phi, theta, psi)
        
        # Kinematic transformation for angular velocities
        T = self._euler_kinematics(phi, theta, psi)
        
        # Mass matrix (including added mass)
        M = self._mass_matrix()
        
        # Coriolis and centripetal matrix
        C = self._coriolis_matrix()
        
        # Damping matrix
        D = self._damping_matrix()
        
        # Restoring forces and moments
        g = self._restoring_forces()
        
        # Position/orientation derivatives
        pos_dot = R @ self.nu[:3]
        ori_dot = T @ self.nu[3:]
        
        # Velocity derivatives
        nu_dot = np.linalg.inv(M) @ (tau - (C + D) @ self.nu - g)
        
        # Update states
        self.eta[:3] += pos_dot * dt  # Update position
        self.eta[3:] += ori_dot * dt  # Update orientation
        self.nu += nu_dot * dt        # Update velocities
        self.time += dt
        
        return self.eta.copy(), self.nu.copy()

    def _mass_matrix(self):
        """Construct the mass matrix including added mass"""
        M = np.diag([
            self.params.mass - self.params.X_udot,
            self.params.mass - self.params.Y_vdot,
            self.params.mass - self.params.Z_wdot,
            self.params.Ixx - self.params.K_pdot,
            self.params.Iyy - self.params.M_qdot,
            self.params.Izz - self.params.N_rdot
        ])
        return M
    
    def _euler_kinematics(self, phi, theta, psi):
        """Kinematic transformation matrix for Euler angles"""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        
        return np.array([
            [1, sphi*sth/cth, cphi*sth/cth],
            [0, cphi, -sphi],
            [0, sphi/cth, cphi/cth]
        ])
    
    def _coriolis_matrix(self):
        """Construct Coriolis-centripetal matrix"""
        C = np.zeros((6, 6))
        nu = self.nu
        
        # Simplified Coriolis terms for ROV
        C[0,4] = -(self.params.mass - self.params.Y_vdot)*nu[2]
        C[0,5] = (self.params.mass - self.params.Z_wdot)*nu[1]
        C[1,3] = (self.params.mass - self.params.X_udot)*nu[2]
        C[1,5] = -(self.params.mass - self.params.Z_wdot)*nu[0]
        C[2,3] = -(self.params.mass - self.params.X_udot)*nu[1]
        C[2,4] = (self.params.mass - self.params.Y_vdot)*nu[0]
        
        # Skew-symmetric property
        C = C - C.T
        return C
    
    def _damping_matrix(self):
        """Construct linear + quadratic damping matrix"""
        nu = self.nu
        D_lin = np.diag([
            -self.params.X_u,
            -self.params.Y_v,
            -self.params.Z_w,
            -self.params.K_p,
            -self.params.M_q,
            -self.params.N_r
        ])
        
        D_nonlin = np.diag([
            -self.params.X_uu*abs(nu[0]),
            -self.params.Y_vv*abs(nu[1]),
            -self.params.Z_ww*abs(nu[2]),
            -self.params.K_pp*abs(nu[3]),
            -self.params.M_qq*abs(nu[4]),
            -self.params.N_rr*abs(nu[5])
        ])
        
        return D_lin + D_nonlin
    
    def _restoring_forces(self):
        """Calculate restoring forces and moments"""
        phi, theta = self.eta[3], self.eta[4]
        g = np.zeros(6)
        
        # Buoyancy and gravity forces
        g[2] = (self.params.weight - self.params.buoyancy)
        
        # Restoring moments
        g[3] = self.params.rg[2] * self.params.weight * np.sin(theta)
        g[4] = -self.params.rg[2] * self.params.weight * np.sin(phi) * np.cos(theta)
        
        return g
    
    def _rotation_matrix(self, phi, theta, psi):
        """Create rotation matrix from Euler angles"""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        return np.array([
            [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi],
            [cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi],
            [-sth, sphi*cth, cphi*cth]
        ])
    
    def reset(self):
        """Reset vehicle state"""
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.time = 0.0