#!/usr/bin/env python3
import numpy as np
import csv
from .pid_controller import PIDController
from ..dynamics.marine_dynamics import MarineDynamics
from ..environment.hull_profiles import HullProfiles
from ..utils.performance_metrics import compute_performance_metrics, calculate_max_error_to_hull

def simulate_with_gains(kp, ki, kd, hull_points, offset_distance=0.0, dt=0.1, max_sim_time=30.0):
    dynamics = MarineDynamics()
    pid = PIDController()
    pid.set_gains(
        [kp, kp, 1.0, 0.5, 0.5, 1.0],
        [ki, ki, 0.05, 0.01, 0.01, 0.05],
        [kd, kd, 0.5, 0.1, 0.1, 0.5]
    )

    # Start at (0, offset) for simulation purposes
    dynamics.eta[0] = 0.0  # x
    dynamics.eta[1] = offset_distance  # y

    trajectory = []
    target_idx = 0
    sim_time = 0

    while sim_time < max_sim_time:
        current_x = dynamics.eta[0]
        if current_x >= 10.0:
            break

        # Target point from hull but controller tracks offset
        raw_target = hull_points[target_idx]
        target_point = np.array([
            raw_target[0],
            raw_target[1] + offset_distance,
            0, 0, 0, 0
        ])

        if np.linalg.norm(dynamics.eta[:2] - target_point[:2]) < 0.2:
            target_idx = min(target_idx + 1, len(hull_points) - 1)

        control = pid.compute(target_point, dynamics.eta)
        tau = np.zeros(6)
        tau[:2] = control[:2]

        dynamics.update(tau, dt)
        trajectory.append(dynamics.eta[:2].copy())
        sim_time += dt

    return np.array(trajectory)

def run_sweeper(
    kp_min=-10.0, kp_max=10.0,
    ki_min=-10.0, ki_max=10.0,
    kd_min=-10.0, kd_max=10.0,
    num_samples=20,
    max_sim_time=30.0,
    offset_distance=0.0
):
    print("Initializing hull points...")
    hull_points = HullProfiles().tanker_hull(length=10.0)
    target = np.array([10.0, 0.0])
    print("Generating gain values...")

    best_models = []
    kp_values = np.linspace(kp_min, kp_max, num_samples)
    ki_values = np.linspace(ki_min, ki_max, num_samples)
    kd_values = np.linspace(kd_min, kd_max, num_samples)

    total_combinations = num_samples ** 3
    print(f"Total combinations to test: {total_combinations}")
    count = 0

    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                count += 1
                print(f"Testing {count}/{total_combinations}: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
                try:
                    trajectory = simulate_with_gains(
                        kp, ki, kd,
                        hull_points=hull_points,
                        offset_distance=offset_distance,
                        max_sim_time=max_sim_time
                    )
                    if trajectory.size == 0:
                        continue

                    time_arr = np.linspace(0, len(trajectory) * 0.1, len(trajectory))
                    metrics = compute_performance_metrics(time_arr, trajectory[:, 1], offset_distance)
                    hull_error = calculate_max_error_to_hull(trajectory, hull_points)
                    metrics['hull_max_y_error'] = hull_error

                    best_models.append((kp, ki, kd, metrics, trajectory))
                except Exception as e:
                    print(f"Simulation error for Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}: {e}")

    print("Sorting top 5 by hull max Y error...")
    best_models.sort(key=lambda x: x[3]['hull_max_y_error'])
    top_5 = best_models[:5]

    results = []
    csv_rows = []
    for idx, (kp, ki, kd, metrics, traj) in enumerate(top_5):
        color = f"C{idx}"
        label = f"Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}"
        print(f"Top {idx+1}: {label} -> Hull Max Y Error: {metrics['hull_max_y_error']:.4f}")
        results.append((traj, color, label))
        csv_rows.append({
            'Rank': idx + 1,
            'Kp': kp,
            'Ki': ki,
            'Kd': kd,
            'Max Error': metrics['max_error'],
            'Hull Max Y Error': metrics['hull_max_y_error'],
            'Overshoot': metrics['overshoot'],
            'Rise Time': metrics['rise_time'],
            'Settling Time': metrics['settling_time']
        })

    csv_path = "top_5_pid_results.csv"
    with open(csv_path, mode='w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_rows[0].keys())
        writer.writeheader()
        writer.writerows(csv_rows)

    print(f"Saved top 5 results to {csv_path}")
    return results, hull_points, offset_distance
