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
        Predict the next n_points based on average velocity from the last 25 smoothed trajectory points,
        starting from the average position of those points.
        :param n_points: Number of future points to predict.
        :return: np.ndarray of shape (n_points, 2)
        """
        smoothed = self.get_smoothed_trajectory()
        n_pts_for_velocity = 25

        if len(smoothed) < 2:
            return np.array([])  # Not enough data to predict

        # Use up to the last n points (or fewer if not enough)
        recent_points = smoothed[-n_pts_for_velocity:]

        # Calculate velocity vectors between consecutive points
        velocities = np.diff(recent_points, axis=0)

        # Average velocity vector
        avg_velocity = np.mean(velocities, axis=0)

        # Average position of recent points (not just last point)
        avg_position = np.mean(recent_points, axis=0)

        # Generate predicted points starting from avg_position using average velocity
        predictions = [avg_position + avg_velocity * (i + 1) for i in range(n_points)]
        return np.array(predictions)
