import unittest
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from aruco_trajectory_estimator import ArucoTrajectoryEstimator

def generate_synthetic_bboxes(path, noise_std=5, width=60, height=60):
    bboxes = []
    for x, y in path:
        noisy_x = x + np.random.normal(0, noise_std)
        noisy_y = y + np.random.normal(0, noise_std)
        bbox = (
            noisy_x - width / 2,
            noisy_y - height / 2,
            noisy_x + width / 2,
            noisy_y + height / 2
        )
        bboxes.append(bbox)
    return bboxes

class TestArucoTrajectoryEstimator(unittest.TestCase):
    def setUp(self):
        self.estimator = ArucoTrajectoryEstimator(window_size=30)
        self.noise_std = 5
        self.extension_points = 30

    def _run_test(self, ground_truth, test_name, equation):
        # Reset estimator
        self.estimator.centroids.clear()

        bboxes = generate_synthetic_bboxes(ground_truth, noise_std=self.noise_std)
        for bbox in bboxes:
            self.estimator.add_bbox(bbox)

        smoothed = self.estimator.get_smoothed_trajectory()
        prediction = self.estimator.predict_trajectory(n_points=self.extension_points)

        # Extend ground truth linearly based on last two GT points
        gt_last_2 = np.array(ground_truth[-2:])
        velocity = gt_last_2[1] - gt_last_2[0]
        gt_extension = np.array([gt_last_2[1] + velocity * (i + 1) for i in range(self.extension_points)])

        # Plot trajectories
        plt.figure(figsize=(10, 6))
        gt_arr = np.array(ground_truth)
        plt.plot(gt_arr[:, 0], gt_arr[:, 1], 'k-', label='Ground Truth')
        plt.plot(smoothed[:, 0], smoothed[:, 1], 'b-', label='Smoothed Trajectory')
        plt.plot(prediction[:, 0], prediction[:, 1], 'g--', label='Predicted Trajectory')
        plt.plot(gt_extension[:, 0], gt_extension[:, 1], 'r-.', label='Extended Ground Truth')
        plt.title(f"Trajectory Prediction Test: {test_name}\nEquation: {equation}")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()

        mse = mean_squared_error(gt_extension, prediction)
        print(f"{test_name} Prediction MSE: {mse:.2f}")
        self.assertLess(mse, 50.0, f"{test_name} Prediction MSE too high: {mse:.2f}")

    def test_linear_diagonal(self):
        gt = [(100 + i * 5, 100 + i * 3) for i in range(30)]
        eq = "y = (3/5) x + b (approx.)"
        self._run_test(gt, "Linear Diagonal", eq)

    def test_horizontal(self):
        gt = [(100 + i * 5, 100) for i in range(30)]
        eq = "y = 100 (constant)"
        self._run_test(gt, "Horizontal Line", eq)

    def test_vertical(self):
        gt = [(100, 100 + i * 5) for i in range(30)]
        eq = "x = 100 (constant)"
        self._run_test(gt, "Vertical Line", eq)

    def test_random_direction(self):
        m = np.random.uniform(-2, 2)
        b = np.random.uniform(50, 150)
        gt = [(x, m * x + b) for x in range(30)]
        eq = f"y = {m:.2f} x + {b:.2f}"
        self._run_test(gt, "Random Direction Line", eq)

    def test_random_line(self):
        m = np.random.uniform(-3, 3)
        b = np.random.uniform(0, 100)
        gt = [(x * 3, m * x * 3 + b) for x in range(30)]
        eq = f"y = {m:.2f} x + {b:.2f}"
        self._run_test(gt, "Random Line", eq)

    def test_parabolic_one_arm(self):
        a = 0.02
        h = 10
        k = 50
        gt = [(x, a * (x - h) ** 2 + k) for x in range(15, 45)]
        eq = f"y = {a} (x - {h})^2 + {k}"
        self._run_test(gt, "Parabolic One Arm", eq)

if __name__ == "__main__":
    unittest.main()
