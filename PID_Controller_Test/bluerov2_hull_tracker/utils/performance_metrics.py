#!/usr/bin/env python3
import numpy as np

def compute_performance_metrics(time, response, target):
    error = response - target
    max_error = np.max(np.abs(error))
    overshoot = (np.max(response) - target) / target * 100 if target != 0 else 0

    rise_time_idx = np.where(response >= 0.9 * target)[0]
    rise_time = time[rise_time_idx[0]] if rise_time_idx.size > 0 else np.nan

    settling_indices = np.where(np.abs(error) > 0.05 * target)[0]
    settling_time = time[settling_indices[-1]] if settling_indices.size > 0 else np.nan

    return {
        "max_error": max_error,
        "overshoot": overshoot,
        "rise_time": rise_time,
        "settling_time": settling_time
    }

def calculate_max_error_to_hull(path, hull_profile):
    """
    Calculate the maximum vertical error (y-distance) from the path to the given hull profile.
    
    Parameters:
        path (np.ndarray): Nx2 array of [x, y] positions representing the trajectory.
        hull_profile (np.ndarray): Mx2 array of [x, y] positions representing the hull profile.
    
    Returns:
        float: Maximum absolute y-distance from the trajectory to the hull.
    """
    # Ensure both arrays are numpy arrays
    path = np.array(path)
    hull_profile = np.array(hull_profile)

    # Sort hull profile by x for interpolation
    hull_profile = hull_profile[np.argsort(hull_profile[:, 0])]

    # Interpolate the hull y-values at the x-coordinates of the path
    hull_y_interp = np.interp(path[:, 0], hull_profile[:, 0], hull_profile[:, 1])

    # Compute the absolute y-error between path and interpolated hull
    y_error = np.abs(path[:, 1] - hull_y_interp)

    return np.max(y_error)