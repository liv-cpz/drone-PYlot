import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
import scipy.signal
from pyr_lucas_kanade import lucas_pyramidal

class EKFVOIMU:
    def __init__(self):
        self.x = np.zeros(9)  # state: [px, py, pz, vx, vy, vz, ba_x, ba_y, ba_z]
        self.P = np.eye(9) * 1.0
        self.Q = np.eye(9) * 0.01
        self.R = np.eye(3) * 0.1
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

    def predict(self, acc, gyro, dt):
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -np.eye(3) * dt  # Bias propagation

        B = np.zeros((9, 6))
        B[3:6, 0:3] = np.eye(3) * dt  # Acceleration
        B[0:3, 3:6] = np.eye(3) * dt  # Gyro (for orientation)

        # Correct IMU measurements with current bias estimates
        true_acc = acc - self.accel_bias
        true_gyro = gyro - self.gyro_bias

        self.x[0:3] += self.x[3:6] * dt + 0.5 * true_acc * dt**2
        self.x[3:6] += true_acc * dt
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(9) - K @ H) @ self.P

    def get_state(self):
        return self.x[:3], self.x[3:6], self.x[6:9]


class VisualOdometryIMU():
    def __init__(self):
        self.K, self.P = self._load_calib("calib.txt")
        self.image_current = None
        self.image_prev = None
        self.cur_pose = np.eye(4)
        
        self.imu_current = None
        self.imu_prev = None

        self.ekf = EKFVOIMU()

        # Define coordinate frames clearly
        # World frame: NED (North-East-Down)
        # Camera frame: Right-Down-Forward (typical computer vision)
        # IMU frame: NED (North-East-Down)
        self.g = np.array([0, 0, -9.81])  # gravity in NED frame

        # IMU integration variables in world frame
        self.acceleration_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.acceleration_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.acceleration_HP = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        self.velocity_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.velocity_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.position_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.position_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Current orientation (rotation from world to body frame)
        self.R_world_to_body = np.eye(3)
        
        self.estimated_path = []
        self.frame_idx = 0
        self.last_time = None

        self.RC = 0.9
       
        # Define transformation matrices clearly
        # IMU to Camera transformation
        # IMU frame: X=forward, Y=right, Z=up
        # Camera frame: X=right, Y=down, Z=forward
        self.R_imu_to_cam = np.array([
            [0,  1,  0],   # x_cam (right) ← y_imu (right)
            [0,  0, -1],   # y_cam (down)  ← -z_imu (up becomes down)
            [1,  0,  0],   # z_cam (forward) ← x_imu (forward)
        ])
        
        # Camera to world transformation (for visualization)
        # This converts from camera frame to world frame
        self.R_cam_to_world = np.array([
            [1,  0,  0],   # x_cam → x_world
            [0, -1,  0],   # y_cam → -y_world (flip Y for display)
            [0,  0, -1],   # z_cam → -z_world (flip Z for display)
        ])

    
    def set_initial_image(self, image):
        self.prev_image = image
    
    def set_initial_pose(self, r, t):
        pose = self._form_transf(r, t) 
        self.cur_pose = pose
        self.R_world_to_body = r.copy()

    def set_initial_imu(self, imu):
        self.imu_current = imu
        self.R_world_to_body = self.euler_to_rotation_matrix(
            imu["roll"], imu["pitch"], imu["yaw"]
        )

    @staticmethod
    def _load_calib(filepath):
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P
    
    @staticmethod
    def _form_transf(R, t):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    
    @staticmethod
    def euler_to_rotation_matrix(roll, pitch, yaw):
        """Returns rotation matrix from world to body frame (Z-Y-X order)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [   -sp,                cp * sr,                cp * cr]
        ])
        return R


    def update_imu(self, dt=0.09):
        """
        Simple approach: use gyroscope for rotation, skip double integration
        """
        dt = 0.09
        alpha = self.RC/(self.RC + dt)

        acc_body = np.array([self.imu_current["agx"],
                                 self.imu_current["agy"],
                                 self.imu_current["agz"]], dtype=np.float64)
        
        R = self.euler_to_rotation_matrix(self.imu_current["roll"],
                                      self.imu_current["pitch"],
                                      self.imu_current["yaw"])
        
         # Transform acceleration from body frame to world frame
        # R_world_to_body.T = R_body_to_world
        acc_world = self.R_world_to_body.T @ acc_body
        self.acceleration_current = acc_world - self.g

        # self.acceleration_current = self.R_imu_to_cam @ self.acceleration_current
        
        accel_hp = alpha * (self.acceleration_HP + self.acceleration_current -  self.acceleration_prev)
        
        self.velocity_world = self.velocity_prev + 0.5*(accel_hp+self.acceleration_HP) * dt
        self.position_world = self.position_prev + self.velocity_prev * dt+0.25 * (accel_hp +self.acceleration_HP) * dt**2
        
        d_vo = self.position_world - self.position_prev
        
        # Update stored values
        self.acceleration_HP = accel_hp
        self.acceleration_prev = self.acceleration_current.copy()
        self.velocity_prev = self.velocity_world.copy()
        self.position_prev = self.position_world.copy()
        

        return np.linalg.norm(d_vo)

    def get_matches(self, img1, img2, number_features=200, wz=5, level=5, number_iteration=70, inlier_threshold=3, static=False):
        q1, q2, _ = lucas_pyramidal(img1, img2, number_features, wz, level, number_iteration, inlier_threshold, static)
        return q1, q2
    
    def get_pose(self, q1, q2):
        E, _ = cv2.findEssentialMat(q1, q2, self.K, method=0, threshold=0.1)
        R, t = self.decomp_essential_mat(E, q1, q2)
        return R, t
    
    def decomp_essential_mat(self, E, q1, q2):
        def sum_z_cal_relative_scale(R, t):
            T = self._form_transf(R, t)
            P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
            hom_Q1 = cv2.triangulatePoints(np.float32(self.P), np.float32(P), np.float32(q1.T), np.float32(q2.T))
            hom_Q2 = np.matmul(T, hom_Q1)
            uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
            return sum(uhom_Q1[2, :] > 0) + sum(uhom_Q2[2, :] > 0)

        R1, R2, t = cv2.decomposeEssentialMat(E)
        t = np.squeeze(t)
        pairs = [[R1, -t], [R1, t], [R2, t], [R2, -t]]
        z_sums = [sum_z_cal_relative_scale(R, t) for R, t in pairs]
        right_pair_idx = np.argmax(z_sums)
        return pairs[right_pair_idx]

    def run(self, image, imu):
        if self.image_prev is None:
            self.image_prev = image
            self.frame_idx += 1
            return
        
        self.image_prev = self.image_current
        self.imu_prev = self.imu_current
        self.image_current = image
        self.imu_current = imu

        # Get visual odometry
        q1, q2 = self.get_matches(self.prev_image, image)
        self.prev_image = image
        
        if q1 is None or q2 is None or len(q1) < 8:
            # Still update IMU even if visual fails
            self.update_imu(0.09)
            return
        
        imu_displacement = self.update_imu(0.09)  # Keep this for future use
        
        R, t = self.get_pose(q1, q2)
        # Firsts transform from camera to body (if needed)
        t_rel_body = self.R_imu_to_cam.T @ t

        t_rel_world = self.R_world_to_body.T @ t_rel_body

        if imu_displacement > 0.001:  # Only use IMU scale if significant motion
            scale = min(imu_displacement, 0.1)  # Cap the scale to avoid outliers
        else:
            scale = 0.01  # Default small scale
            
        t_scaled_world = t_rel_world * scale

        # Transform rotation to world frame
        R_rel_body = self.R_imu_to_cam.T @ R @ self.R_imu_to_cam
        R_rel_world = self.R_world_to_body.T @ R_rel_body @ self.R_world_to_body

        # Update global pose
        # Convert world frame motion back to camera frame for pose update
        t_cam_for_pose = self.R_cam_to_world.T @ t_scaled_world
        R_cam_for_pose = self.R_cam_to_world.T @ R_rel_world @ self.R_cam_to_world
        
        # Create transformation matrix
        transf = self._form_transf(R_cam_for_pose, t_cam_for_pose)
        self.cur_pose = np.matmul(self.cur_pose, np.linalg.inv(transf))

        # Store trajectory (convert to display coordinates)
        pos_display = self.R_cam_to_world @ self.cur_pose[:3, 3]
        R_display = self.R_cam_to_world @ self.cur_pose[:3, :3]
        # pos_display = self.cur_pose[:3, 3]
        # R_display = self.cur_pose[:3, :3]
        
        estimated_rotation, _ = cv2.Rodrigues(R_display)
        self.estimated_path.append((
            estimated_rotation[0, 0],
            estimated_rotation[1, 0], 
            estimated_rotation[2, 0],
            pos_display[0],
            pos_display[1],
            pos_display[2]
        ))
        

    def get_current_pose(self):
        if self.cur_pose is not None:
            return self.cur_pose.copy()
        return None
    
    def get_estimated_path(self):
        return np.array(self.estimated_path) if self.estimated_path else None