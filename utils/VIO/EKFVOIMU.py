import numpy as np
from scipy.spatial.transform import Rotation as R

class EKFVOIMU:
    def __init__(self):
        self.x = np.zeros(10)  # position, velocity, quaternion
        self.x[6] = 1.0  # initial orientation quaternion (w=1)

        self.P = np.eye(10) * 0.01  # initial uncertainty

        self.Q = np.eye(10) * 0.01  # process noise
        self.R_vo = np.eye(6) * 0.1  # VO measurement noise (position + orientation)

        self.g = np.array([0, 0, -9.81])  # gravity

    def predict(self, acc, gyro, dt):
        # Convert orientation to rotation matrix
        q = self.x[6:10]
        rot = R.from_quat(q)
        Rwb = rot.as_matrix()

        # Acceleration in world frame
        acc_world = Rwb @ acc + self.g

        # Integrate velocity and position
        self.x[0:3] += self.x[3:6] * dt + 0.5 * acc_world * dt**2
        self.x[3:6] += acc_world * dt

        # Integrate orientation using gyro
        omega = gyro * dt
        dq = R.from_rotvec(omega).as_quat()
        new_q = R.from_quat(q) * R.from_quat(dq)
        self.x[6:10] = new_q.as_quat()

        # Normalize quaternion
        self.x[6:10] /= np.linalg.norm(self.x[6:10])

        # TODO: Add Jacobian and propagate P
        self.P += self.Q

    def update_vo(self, position_meas, orientation_meas_quat):
        # Innovation
        pos_pred = self.x[0:3]
        ori_pred = self.x[6:10]
        delta_pos = position_meas - pos_pred

        delta_ori = R.from_quat(orientation_meas_quat) * R.from_quat(ori_pred).inv()
        delta_ori_vec = delta_ori.as_rotvec()

        z = np.hstack((delta_pos, delta_ori_vec))

        # Kalman gain
        H = np.zeros((6, 10))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)  # approximate for small angle

        S = H @ self.P @ H.T + self.R_vo
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        dx = K @ z
        self.x[0:3] += dx[0:3]
        self.x[3:6] += dx[3:6]  # velocity update optional

        dtheta = dx[3:6]
        dq = R.from_rotvec(dtheta).as_quat()
        updated_q = R.from_quat(self.x[6:10]) * R.from_quat(dq)
        self.x[6:10] = updated_q.as_quat()
        self.x[6:10] /= np.linalg.norm(self.x[6:10])

        # Covariance update
        I = np.eye(10)
        self.P = (I - K @ H) @ self.P
