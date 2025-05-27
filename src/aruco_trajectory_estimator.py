#!/usr/bin/env python3
import numpy as np
from scipy.signal import savgol_filter
from collections import deque
from typing import Optional, Tuple, List
import rclpy
from rclpy.time import Time

class ArucoTrajectoryEstimator:
    def __init__(self, window_size: int = 30, poly_order: int = 2):
        """
        Initialize the AruCo trajectory estimator.
        
        Args:
            window_size: Number of historical points to maintain
            poly_order: Polynomial order for smoothing and prediction
        """
        self.window_size = window_size
        self.poly_order = poly_order
        
        # Raw trajectory storage (center points in image coordinates)
        self.trajectory = deque(maxlen=window_size)
        
        # Timestamps for velocity estimation
        self.timestamps = deque(maxlen=window_size)
        
        # Marker corners storage for orientation estimation
        self.corners_history = deque(maxlen=window_size)
        
        # Marker IDs (for multiple marker tracking)
        self.marker_ids = deque(maxlen=window_size)

    def update(self, marker_id: int, corners: np.ndarray, timestamp: Optional[Time] = None) -> None:
        """
        Update the estimator with a new AruCo marker observation.
        
        Args:
            marker_id: ID of the detected marker
            corners: 4x2 array of marker corners in image coordinates
            timestamp: ROS2 timestamp of the observation
        """
        if corners is None or corners.shape != (4, 2):
            return
            
        # Calculate center point
        center = np.mean(corners, axis=0)
        
        # Store trajectory data
        self.trajectory.append(center)
        self.corners_history.append(corners)
        self.marker_ids.append(marker_id)
        
        # Store timestamp or use current time if not provided
        if timestamp is None:
            self.timestamps.append(Time().nanoseconds)
        else:
            self.timestamps.append(timestamp.nanoseconds)

    def get_smoothed_trajectory(self) -> np.ndarray:
        """
        Apply Savitzky-Golay filter to smooth the trajectory.
        
        Returns:
            Nx2 array of smoothed (x,y) positions
        """
        if len(self.trajectory) < 5:
            return np.array(self.trajectory)
            
        traj_array = np.array(self.trajectory)
        
        # Ensure window length is odd and smaller than trajectory length
        window_length = min(len(self.trajectory) - 1, 11)  # Max window of 11
        window_length = window_length if window_length % 2 == 1 else window_length - 1
        
        if window_length < 3:
            return traj_array
            
        # Smooth x and y coordinates separately
        x_smooth = savgol_filter(traj_array[:, 0], window_length, self.poly_order)
        y_smooth = savgol_filter(traj_array[:, 1], window_length, self.poly_order)
        
        return np.column_stack((x_smooth, y_smooth))

    def predict_future_positions(self, steps: int = 5) -> Optional[np.ndarray]:
        """
        Predict future positions using polynomial extrapolation.
        
        Args:
            steps: Number of future steps to predict
            
        Returns:
            stepsx2 array of predicted (x,y) positions, or None if insufficient data
        """
        if len(self.trajectory) < 3:
            return None
            
        smoothed = self.get_smoothed_trajectory()
        t = np.arange(len(smoothed))
        
        # Fit polynomials to smoothed trajectory
        x_coeffs = np.polyfit(t, smoothed[:, 0], deg=self.poly_order)
        y_coeffs = np.polyfit(t, smoothed[:, 1], deg=self.poly_order)
        
        # Predict future points
        future_t = np.arange(len(t), len(t)+steps)
        x_pred = np.polyval(x_coeffs, future_t)
        y_pred = np.polyval(y_coeffs, future_t)
        
        return np.column_stack((x_pred, y_pred))

    def estimate_velocity(self) -> Tuple[float, float]:
        """
        Estimate current velocity in pixels/sec.
        
        Returns:
            (vx, vy) velocity components
        """
        if len(self.trajectory) < 2:
            return (0.0, 0.0)
            
        # Calculate time difference in seconds
        dt_ns = self.timestamps[-1] - self.timestamps[-2]
        if dt_ns <= 0:
            return (0.0, 0.0)
        dt = dt_ns * 1e-9
        
        # Calculate pixel difference
        dx = self.trajectory[-1][0] - self.trajectory[-2][0]
        dy = self.trajectory[-1][1] - self.trajectory[-2][1]
        
        return (dx/dt, dy/dt)

    def estimate_marker_orientation(self) -> Optional[float]:
        """
        Estimate marker orientation angle in radians.
        
        Returns:
            Orientation angle in radians (0 to 2π), or None if insufficient data
        """
        if len(self.corners_history) == 0:
            return None
            
        # Use most recent corners
        corners = self.corners_history[-1]
        
        # Calculate vectors between opposite corners
        vec1 = corners[1] - corners[0]
        vec2 = corners[2] - corners[1]
        
        # Average orientation from two edges
        angle1 = np.arctan2(vec1[1], vec1[0])
        angle2 = np.arctan2(vec2[1], vec2[0])
        avg_angle = (angle1 + angle2) / 2
        
        return avg_angle % (2*np.pi)

    def calculate_marker_size(self) -> Optional[float]:
        """
        Calculate the approximate size of the marker in pixels.
        
        Returns:
            Average edge length in pixels, or None if no data
        """
        if len(self.corners_history) == 0:
            return None
            
        corners = self.corners_history[-1]
        
        # Calculate lengths of all four edges
        edge_lengths = [
            np.linalg.norm(corners[1] - corners[0]),
            np.linalg.norm(corners[2] - corners[1]),
            np.linalg.norm(corners[3] - corners[2]),
            np.linalg.norm(corners[0] - corners[3])
        ]
        
        return np.mean(edge_lengths)

    def is_marker_rotating(self, window: int = 5) -> Optional[bool]:
        """
        Detect if the marker is rotating based on orientation changes.
        
        Args:
            window: Number of recent frames to consider
            
        Returns:
            True if rotating, False if stable, None if insufficient data
        """
        if len(self.corners_history) < window:
            return None
            
        # Get recent orientations
        orientations = []
        for i in range(-window, 0):
            corners = self.corners_history[i]
            vec = corners[1] - corners[0]
            angle = np.arctan2(vec[1], vec[0])
            orientations.append(angle)
        
        # Calculate angular differences
        diffs = np.abs(np.diff(orientations))
        mean_diff = np.mean(diffs)
        
        return mean_diff > 0.1  # Threshold in radians

    def calculate_confidence(self) -> float:
        """
        Calculate a confidence score (0-1) in the current tracking.
        
        Returns:
            Confidence score based on recent tracking consistency
        """
        if len(self.trajectory) < 2:
            return 0.0
            
        # Calculate velocity consistency
        velocities = []
        for i in range(1, len(self.trajectory)):
            dt_ns = self.timestamps[i] - self.timestamps[i-1]
            if dt_ns <= 0:
                continue
            dt = dt_ns * 1e-9
            dx = self.trajectory[i][0] - self.trajectory[i-1][0]
            dy = self.trajectory[i][1] - self.trajectory[i-1][1]
            velocities.append((dx/dt, dy/dt))
        
        if len(velocities) < 2:
            return 0.5
            
        # Calculate velocity variance
        vel_array = np.array(velocities)
        var = np.mean(np.var(vel_array, axis=0))
        
        # Convert to confidence (lower variance = higher confidence)
        return np.exp(-var / 100.0)  # Adjust denominator as needed

    def reset(self) -> None:
        """Reset the estimator to initial state."""
        self.trajectory.clear()
        self.timestamps.clear()
        self.corners_history.clear()
        self.marker_ids.clear()

    def get_trajectory_length(self) -> float:
        """
        Calculate the total length of the recorded trajectory.
        
        Returns:
            Total path length in pixels
        """
        if len(self.trajectory) < 2:
            return 0.0
            
        length = 0.0
        for i in range(1, len(self.trajectory)):
            dx = self.trajectory[i][0] - self.trajectory[i-1][0]
            dy = self.trajectory[i][1] - self.trajectory[i-1][1]
            length += np.sqrt(dx**2 + dy**2)
            
        return length

    def get_recent_marker_ids(self, n: int = 5) -> List[int]:
        """
        Get the most recent marker IDs observed.
        
        Args:
            n: Number of recent IDs to return
            
        Returns:
            List of marker IDs (may contain duplicates)
        """
        return list(self.marker_ids)[-n:]