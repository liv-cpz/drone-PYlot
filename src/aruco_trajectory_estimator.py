import numpy as np
from scipy.signal import savgol_filter

class ArucoTrajectoryEstimator:
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.centroids = []

    def add_bbox(self, bbox):
        """
        Add a bounding box defined by (x_min, y_min, x_max, y_max).
        :param bbox: Tuple of 4 coordinates
        """
        x_min, y_min, x_max, y_max = bbox
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        self.centroids.append((cx, cy))

        if len(self.centroids) > self.window_size:
            self.centroids.pop(0)

    def get_smoothed_trajectory(self):
        """
        Return a smoothed 2D trajectory using Savitzky-Golay filter.
        :return: np.ndarray of shape (N, 2)
        """
        if len(self.centroids) < 5:
            return np.array(self.centroids)  # Not enough for smoothing

        coords = np.array(self.centroids)
        x_smooth = savgol_filter(coords[:, 0], window_length=5, polyorder=2)
        y_smooth = savgol_filter(coords[:, 1], window_length=5, polyorder=2)
        return np.stack((x_smooth, y_smooth), axis=1)

    def get_raw_trajectory(self):
        return np.array(self.centroids)

    def predict_trajectory(self, n_points=30):
        """
        Predict the next n_points based on linear projection of the smoothed trajectory.
        :param n_points: Number of future points to predict.
        :return: np.ndarray of shape (n_points, 2)
        """
        smoothed = self.get_smoothed_trajectory()
        
        if len(smoothed) < 2:
            return np.array([])  # Need at least 2 points for linear projection

        # Create time indices
        t = np.arange(len(smoothed))
        t_future = np.arange(len(smoothed), len(smoothed) + n_points)
        
        # Linear fit for x coordinates (slope and intercept)
        slope_x, intercept_x = np.polyfit(t, smoothed[:, 0], 1)
        x_pred = intercept_x + slope_x * t_future
        
        # Linear fit for y coordinates (slope and intercept)
        slope_y, intercept_y = np.polyfit(t, smoothed[:, 1], 1)
        y_pred = intercept_y + slope_y * t_future
        
        return np.stack((x_pred, y_pred), axis=1)