# Corrected Visual-Inertial Odometry System
import time
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from pyr_lucas_kanade import lucas_pyramidal

class VIO():
    def __init__(self, data_dir=None):
        # Default camera parameters - you should replace with actual calibration
        # self.K = np.array([[921.170702, 0.000000, 459.904354],
        #                   [0.000000, 919.018377, 351.238301],
        #                   [0.000000, 0.000000, 1.000000]], dtype=np.float32)
        
        # # Create projection matrix
        # self.P = np.hstack((self.K, np.zeros((3, 1))))
        
        self.K, self.P = self._load_calib("calib.txt")

        # Image and IMU data
        self.image_prev = None
        self.imu_prev = None
        self.image_current = None
        self.imu_current = None
        
        # Feature tracking parameters
        self.number_features = 500
        self.wz = 5
        self.level = 5
        self.number_iteration = 70
        self.inlier_threshold = 3
        self.static = False
        
        # Pose and velocity tracking
        self.cur_pose = np.eye(4)
        self.velocity = np.zeros(3)  # Current velocity
        self.estimated_path = []
        
        # Time tracking
        self.prev_time = None
        
        # IMU bias estimation and processing
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.bias_initialized = False
        self.bias_samples = []
        self.max_bias_samples = 20
        
        # Scale and motion management
        self.scale_factor = 1.0
        self.scale_initialized = False
        self.motion_threshold = 0.001  # Minimum motion to update pose
        
        # Gravity vector (NED frame)
        self.gravity = np.array([0.0, 0.0, -9.81])
        
        # Simple Kalman filter states for IMU integration
        self.imu_position = np.zeros(3)
        self.imu_velocity = np.zeros(3)
        
        # Complementary filter weight for sensor fusion
        self.visual_weight = 0.9
        self.imu_weight = 0.1

        self.imu_displacement_buffer = []
        self.window_size = 5
    
    def set_initial_image(self, image):
        self.image_current = image
    
    def set_initial_pose(self, r, t):
        pose = self._form_transf(r, t) 
        self.cur_pose = pose
        # Store initial position
        self.estimated_path.append([0, 0, 0, t[0], t[1], t[2]])

    def set_initial_imu(self, imu):
        self.imu_current = imu
        self.prev_time = time.time()
        # Start collecting bias samples
        self.bias_samples.append([
            imu["agx"], imu["agy"], imu["agz"],
            imu.get("roll", 0), imu.get("pitch", 0), imu.get("yaw", 0)
        ])

    @staticmethod
    def _load_calib(filepath):
        """
        Loads the calibration of the camera
        Parameters
        ----------
        filepath (str): The file path to the camera file

        Returns
        -------
        K (ndarray): Intrinsic parameters
        P (ndarray): Projection matrix
        """
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P

    @staticmethod
    def _form_transf(R, t):
        """Makes a transformation matrix from the given rotation matrix and translation vector"""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    
    def estimate_imu_bias(self):
        """Estimate IMU bias from initial stationary samples"""
        if len(self.bias_samples) >= self.max_bias_samples and not self.bias_initialized:
            bias_array = np.array(self.bias_samples)
            # Estimate accelerometer bias (remove gravity component)
            self.accel_bias = np.mean(bias_array[:, :3], axis=0)
            # The Z acceleration should be close to -g when stationary
            self.accel_bias -= self.gravity  # Remove gravity
            
            self.bias_initialized = True
            print(f"IMU bias estimated: accel={self.accel_bias}")
    
    def integrate_imu(self, imu, dt):
        """
        Integrate IMU measurements to estimate motion
        """
        if not self.bias_initialized:
            return np.zeros(3), np.zeros(3)
        
        # Remove bias from accelerometer readings
        accel = np.array([imu["agx"], imu["agy"], imu["agz"]]) - self.accel_bias
        
        # Transform acceleration to world frame (simplified - assumes small rotations)
        # In a full implementation, you'd use the current orientation estimate
        rotation_world_from_body = self.cur_pose[:3, :3]
        world_accel = rotation_world_from_body @ accel + self.gravity

        
        # Integrate acceleration to get velocity and position changes
        delta_velocity = world_accel * dt
        delta_position = self.imu_velocity * dt + 0.5 * world_accel * dt**2
        
        # Update IMU velocity
        self.imu_velocity += delta_velocity
        
        # Apply simple damping to prevent drift
        # self.imu_velocity *= 0.95
        
        return delta_position, delta_velocity

    def estimate_scale_from_imu(self, visual_translation, imu_delta_pos, dt):
        """
        Estimate scale using IMU measurements
        """
        visual_magnitude = np.linalg.norm(visual_translation)
        imu_magnitude = np.linalg.norm(imu_delta_pos)
        
        self.imu_displacement_buffer.append(imu_magnitude)
        if len(self.imu_displacement_buffer) > self.window_size:
            self.imu_displacement_buffer.pop(0)

        smoothed_imu_disp = np.mean(self.imu_displacement_buffer)

        if visual_magnitude > 1e-6:
            scale = smoothed_imu_disp / visual_magnitude
            return np.clip(scale, 0.01, 2.0)
        return 1.0
    

    def get_matches(self):
        """Detect and track features using Lucas-Kanade optical flow"""
        if self.image_prev is None:
            return None, None
            
        try:
            q1, q2, _ = lucas_pyramidal(
                self.image_prev, self.image_current, 
                self.number_features, self.wz, self.level,
                self.number_iteration, self.inlier_threshold, 
                self.static
            )
            return q1, q2
        except Exception as e:
            print(f"Feature matching failed: {e}")
            return None, None

    def get_pose(self, q1, q2):
        """Calculate relative pose from feature matches"""
        if q1 is None or q2 is None or len(q1) < 8:
            return np.eye(3), np.zeros(3)
        
        try:
            # Find essential matrix
            E, mask = cv2.findEssentialMat(
                q1, q2, self.K, 
                method=cv2.RANSAC, 
                prob=0.999, 
                threshold=1.0,
                maxIters=1000
            )
            
            if E is None or mask is None:
                return np.eye(3), np.zeros(3)
            
            # Use only inlier points
            q1_inliers = q1[mask.ravel() == 1]
            q2_inliers = q2[mask.ravel() == 1]
            
            if len(q1_inliers) < 8:
                return np.eye(3), np.zeros(3)
            
            # Decompose essential matrix
            R, t = self.decomp_essential_mat(E, q1_inliers, q2_inliers)
            
            return R, t
            
        except Exception as e:
            print(f"Pose estimation failed: {e}")
            return np.eye(3), np.zeros(3)

    def decomp_essential_mat(self, E, q1, q2):
        """Decompose the Essential matrix and find the correct solution"""
        def count_positive_depth(R, t):
            try:
                T = self._form_transf(R, t)
                P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
                
                hom_Q1 = cv2.triangulatePoints(
                    np.float32(self.P), np.float32(P), 
                    np.float32(q1.T), np.float32(q2.T)
                )
                
                hom_Q2 = np.matmul(T, hom_Q1)
                
                uhom_Q1 = hom_Q1[:3, :] / (hom_Q1[3, :] + 1e-8)
                uhom_Q2 = hom_Q2[:3, :] / (hom_Q2[3, :] + 1e-8)

                return np.sum(uhom_Q1[2, :] > 0) + np.sum(uhom_Q2[2, :] > 0)
            except:
                return 0

        try:
            R1, R2, t = cv2.decomposeEssentialMat(E)
            t = np.squeeze(t)
            
            pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]
            scores = [count_positive_depth(R, t_test) for R, t_test in pairs]
            
            if max(scores) > 0:
                best_idx = np.argmax(scores)
                return pairs[best_idx]
            else:
                return np.eye(3), np.zeros(3)
                
        except Exception as e:
            print(f"Essential matrix decomposition failed: {e}")
            return np.eye(3), np.zeros(3)

    def run(self, image, imu, timestamp=None):
        """Main VIO processing function with sensor fusion"""
        # Update previous states
        self.image_prev = self.image_current
        self.imu_prev = self.imu_current
        
        # Set current states
        self.image_current = image
        self.imu_current = imu
        
        # Collect bias samples if not initialized
        if not self.bias_initialized:
            if len(self.bias_samples) < self.max_bias_samples:
                self.bias_samples.append([
                    imu["agx"], imu["agy"], imu["agz"],
                    imu.get("roll", 0), imu.get("pitch", 0), imu.get("yaw", 0)
                ])
            else:
                self.estimate_imu_bias()
        
        # Calculate time interval
        current_time = time.time() if timestamp is None else timestamp
        dt = 0.033 if self.prev_time is None else current_time - self.prev_time
        self.prev_time = current_time
        dt = np.clip(dt, 0.01, 0.1)
        
        # Integrate IMU measurements
        imu_delta_pos, imu_delta_vel = self.integrate_imu(imu, dt)
        
        # Get feature matches and visual pose
        q1, q2 = self.get_matches()
        visual_translation = np.zeros(3)
        visual_rotation = np.eye(3)

        if q1 is not None and q2 is not None and len(q1) >= 8:
            R, t = self.get_pose(q1, q2)
            if np.linalg.norm(t) > self.motion_threshold:
                visual_rotation = R
                scale = self.estimate_scale_from_imu(t, imu_delta_pos, dt)
                
                # Convert visual translation from camera to NED
                # R_cam_to_ned = np.array([
                #     [0, 0, 1],   # X_body = Z_cam
                #     [1, 0, 0],   # Y_body = X_cam
                #     [0, -1, 0]   # Z_body = -Y_cam
                # ])
                R_cam_to_ned = np.array([
                    [0, 0, 1],   # cam_x → ned_z
                    [-1, 0, 0],  # cam_y → -ned_x
                    [0, -1, 0]   # cam_z → -ned_y
                ])

                visual_translation = R_cam_to_ned @ (t * scale)
                visual_rotation = R_cam_to_ned @ visual_rotation

                # print(f"Visual translation (camera frame): {t}")
                # print(f"Visual translation (scaled): {t * scale}")
                # print(f"Visual translation (body/NED frame): {visual_translation}")


        if np.linalg.norm(visual_translation) > self.motion_threshold:
            fused_translation = (self.visual_weight * visual_translation + 
                                 self.imu_weight * imu_delta_pos)
        else:
            fused_translation = imu_delta_pos
            visual_rotation = np.eye(3)

        if np.linalg.norm(fused_translation) > self.motion_threshold:
            delta_transform = self._form_transf(visual_rotation, fused_translation)
            self.cur_pose = np.matmul(self.cur_pose, delta_transform)

        if self.cur_pose is not None:
            rotation_matrix = self.cur_pose[:3, :3]
            rotation_vec, _ = cv2.Rodrigues(rotation_matrix)
            position = self.cur_pose[:3, 3]
            self.estimated_path.append([
                rotation_vec[0, 0],
                rotation_vec[1, 0], 
                rotation_vec[2, 0], 
                position[0], 
                position[1], 
                position[2]
            ])
        

    def get_current_pose(self):
        if self.cur_pose is not None:
            return self.cur_pose.copy()
        return None
    
    def get_estimated_path(self):
        return np.array(self.estimated_path) if self.estimated_path else Nones