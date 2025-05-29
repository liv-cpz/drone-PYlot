# # TODO Modify Visual Odometry to work with live data
# # TODO integrate IMU data? via Kalman Filter?
# # TODO imu data check 
# # TODO inital pose 
# # Kalman filter on speed + acceleration data


# # VIO
# import time
# import numpy as np
# import cv2

# from scipy.spatial.transform import Rotation
# from pyr_lucas_kanade import lucas_pyramidal

# class VIO():
#     def __init__(self, data_dir=None):
#         # self.K, self.P = self._load_calib("calib.txt")
#         self.K = np.array([[921.170702, 0.000000, 459.904354],
#                           [0.000000, 919.018377, 351.238301],
#                           [0.000000, 0.000000, 1.000000]], dtype=np.float32)
        
#         # Create projection matrix
#         self.P = np.hstack((self.K, np.zeros((3, 1))))

#         self.image_prev = None
#         self.imu_prev = None

#         self.image_current = None
#         self.imu_current = None

#         # IMU integration variables - explicitly initialize all
#         self.velocity_current = np.array([0.0, 0.0, 0.0], dtype=np.float64)
#         self.velocity_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)
#         self.position_current = np.array([0.0, 0.0, 0.0], dtype=np.float64)
#         self.position_prev = np.array([0.0, 0.0, 0.0], dtype=np.float64)

#         # Feature tracking Parameters
#         self.number_features = 500
#         self.wz = 5
#         self.level = 5
#         self.number_iteration=70
#         self.inlier_threshold=3
#         self.static = False
#         # TODO set inital cur_pose

#         self.cur_pose = None

#         self.estimated_path = []
#         # For time tracking
#         self.prev_time = None
    
#     def set_initial_image(self, image):
#         self.image_current = image
    
#     def set_initial_pose(self, r,t):
#         pose = self._form_transf(r, t) 
#         self.cur_pose = pose

#     def set_initial_imu(self, imu):
#         self.imu_current = imu
#         self.prev_time = time.time()

#     @staticmethod
#     def _load_calib(filepath):
#         """
#         Loads the calibration of the camera
#         Parameters
#         ----------
#         filepath (str): The file path to the camera file

#         Returns
#         -------
#         K (ndarray): Intrinsic parameters
#         P (ndarray): Projection matrix
#         """
#         with open(filepath, 'r') as f:
#             params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
#             P = np.reshape(params, (3, 4))
#             K = P[0:3, 0:3]
#         return K, P

#     @staticmethod
#     def _form_transf(R, t):
#         """
#         Makes a transformation matrix from the given rotation matrix and translation vector

#         Parameters
#         ----------
#         R (ndarray): The rotation matrix
#         t (list): The translation vector

#         Returns
#         -------
#         T (ndarray): The transformation matrix
#         """
#         T = np.eye(4, dtype=np.float64)
#         T[:3, :3] = R
#         T[:3, 3] = t
#         return T
    

#     def undistort(self):
#         pass

#     def integrate(self, dt):
#         # TODO check this --> Kalman filter this?
#         # Numerical integration to calculate velocity
#         delta_vx = (self.imu_current["agx"]+
#                     self.imu_current["agy"] + 
#                     self.imu_prev["agx"]+ 
#                     self.imu_prev["agy"])/2
#         delta_vy = (self.imu_current["agy"]-self.imu_current["agy"] + 
#                     self.imu_prev["agy"]-self.imu_prev["agy"])/2
#         delta_vz = ((-self.imu_current["agz"] -self.imu_current["agy"])
#                     + (-self.imu_prev["agz"]-self.imu_prev["agy"]))/2

#         delta_vx = delta_vx*dt
#         delta_vy = delta_vy*dt
#         delta_vz = delta_vz*dt

#         self.velocity_current = self.velocity_prev + np.array([delta_vx, delta_vy, delta_vz])

#         # self.velocity_current = np.array([self.imu_current["vgx"],
#         #                                   self.imu_current["vgy"],
#         #                                   self.imu_current["vgz"]])

#         # Numerical integration to calculate displacement 
#         delta_d = ((self.velocity_current + self.velocity_prev)/2)*dt

#         self.distance_current = delta_d+self.distance_prev
#         self.velocity_prev = self.velocity_current
#         self.distance_prev = self.distance_current

#         return np.linalg.norm(delta_d)

#     def integrate_imu(self, dt):
#         """
#         Fixed IMU integration with proper acceleration handling
#         """
#         if self.imu_prev is None:
#             return 0.0
        
#         # Get accelerations (assuming they're in m/s²)
#         # Note: Tello IMU might need different scaling/offset handling
#         accel_current = np.array([
#             self.imu_current["agx"],
#             self.imu_current["agy"], 
#             self.imu_current["agz"]
#         ])
        
#         accel_prev = np.array([
#             self.imu_prev["agx"],
#             self.imu_prev["agy"],
#             self.imu_prev["agz"]
#         ])
        
#         # Trapezoidal integration for velocity
#         accel_avg = (accel_current + accel_prev) / 2.0
#         delta_v = accel_avg * dt
#         self.velocity_current = self.velocity_prev + delta_v
        
#         # Trapezoidal integration for position
#         velocity_avg = (self.velocity_current + self.velocity_prev) / 2.0
#         delta_pos = velocity_avg * dt
#         self.position_current = self.position_prev + delta_pos
        
#         # Update previous values
#         self.velocity_prev = self.velocity_current.copy()
#         self.position_prev = self.position_current.copy()
        
#         # Return displacement magnitude for scale estimation
#         displacement_magnitude = np.linalg.norm(delta_pos)
#         return displacement_magnitude
    
#     def get_matches(self):
#         """
#         This function detect and compute keypoints from the i-1'th and i'th image using the Lucas Kanade optical flow function

#         Parameters
#         ----------
#         i (int): The current frame

#         Returns
#         -------
#         q1 (ndarray): The good keypoints position in i-1'th image
#         q2 (ndarray): The good keypoints position in i'th image
#         """
#         if self.image_prev is None:
#             return None, None

#         q1 , q2, foes = lucas_pyramidal(self.image_prev, self.image_current, 
#                                         self.number_features, self.wz , self.level ,
#                                         self.number_iteration, self.inlier_threshold, 
#                                         self.static)

#         """This function plots the optical flow and on the i'th image"""
#         # key = plot(self.images[i-step], np.copy(q1), np.copy(q2), foes, static)
#         #print(time1-time0)
#         #key = 0
#         # return q1, q2, key
#         return q1, q2


#     def get_pose(self, q1, q2):
#         # z_c is the distance between the center of the camera and the center of the optitrack markers. This is an approximate that was measured using a ruler
#         """
#         Calculates the transformation matrix

#         Parameters
#         ----------
#         q1 (ndarray): The good keypoints matches position in i-1'th image
#         q2 (ndarray): The good keypoints matches position in i'th image

#         Returns
#         -------
#         transformation_matrix (ndarray): The transformation matrix
#         """
#         if q1 is None or q2 is None or len(q1) < 8:
#             return np.eye(3), np.zeros(3)

#         # Essential matrix
#         E, _ = cv2.findEssentialMat(q1, q2, self.K, 
#                                     method=cv2.RANSAC,
#                                     prob=0.999,
#                                     threshold=0.1)

#         if E is None:
#             return np.eye(3), np.zeros(3)
        
#         # Decompose the Essential matrix into R and t
#         R, t = self.decomp_essential_mat(E, q1, q2)

#         # Get transformation matrix
#         # transformation_matrix = self._form_transf(R, np.squeeze(t))
#         return R, t

#     def decomp_essential_mat(self, E, q1, q2):
#         """
#         Decompose the Essential matrix

#         Parameters
#         ----------
#         E (ndarray): Essential matrix
#         q1 (ndarray): The good keypoints matches position in i-1'th image
#         q2 (ndarray): The good keypoints matches position in i'th image

#         Returns
#         -------
#         right_pair (list): Contains the rotation matrix and translation vector
#         """
#         def sum_z_cal_relative_scale(R, t):
#             # Get the transformation matrix
#             T = self._form_transf(R, t)
#             # Make the projection matrix
#             P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
            
#             # Triangulate the 3D points
#             hom_Q1 = cv2.triangulatePoints(np.float32(self.P), np.float32(P), np.float32(q1.T), np.float32(q2.T))
            
#             # Also seen from cam 2
#             hom_Q2 = np.matmul(T, hom_Q1)

#             # # Un-homogenize
#             # uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
#             # uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
#              # Un-homogenize
#             uhom_Q1 = hom_Q1[:3, :] / (hom_Q1[3, :] + 1e-8)
#             uhom_Q2 = hom_Q2[:3, :] / (hom_Q2[3, :] + 1e-8)


#             # Find the number of points there has positive z coordinate in both cameras
#             sum_of_pos_z_Q1 = sum(uhom_Q1[2, :] > 0)
#             sum_of_pos_z_Q2 = sum(uhom_Q2[2, :] > 0)

#             return sum_of_pos_z_Q1 + sum_of_pos_z_Q2

#         # Decompose the essential matrix
#         R1, R2, t = cv2.decomposeEssentialMat(E)
#         t = np.squeeze(t)
#         #t = t*z_c
        
#         # Make a list of the different possible pairs
#         pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]

#         # Check which solution there is the right one
#         z_sums = []
#         relative_scales = []
#         for R, t in pairs:
#             z_sum = sum_z_cal_relative_scale(R, t)
#             z_sums.append(z_sum)

#         # Select the best solution
#         if len(z_sums) > 0:
#             best_idx = np.argmax(z_sums)
#             R_best, t_best = pairs[best_idx]
#         else:
#             R_best, t_best = np.eye(3), np.zeros(3)
        
#         return R_best, t_best


#     def run(self, image, imu, timestamp=None):
#         #time interval between frames 

#         self.image_prev = self.image_current
#         self.imu_prev = self.imu_current
#         self.image_current = image
#         self.imu_current = imu # dictionary 

#         # Calculate time interval
#         current_time = time.time() if timestamp is None else timestamp
#         if self.prev_time is not None:
#             dt = current_time - self.prev_time
#         else:
#             dt = 0.033  # Default ~30 FPS
#         self.prev_time = current_time


#         # Ensure reasonable dt bounds
#         dt = np.clip(dt, 0.01, 0.1)

#         q1, q2 = self.get_matches()
        
#         if q1 is not None and q2 is not None and len(q1) >= 8:
#             # Estimate visual motion
#             R, t = self.get_pose(q1, q2)
            
#             # Get IMU-based scale estimate
#             imu_displacement = self.integrate_imu(dt)
            
#             # Scale the translation with IMU information
#             # This is a simple approach - more sophisticated fusion could be used
#             if np.linalg.norm(t) > 1e-6:
#                 scale_factor = max(imu_displacement, 0.001)  # Prevent division by zero
#                 t_scaled = t * scale_factor
#             else:
#                 t_scaled = t
            
#             # Create transformation matrix
#             transf = self._form_transf(R, t_scaled)
            
#             # Update pose (integrate motion)
#             if self.cur_pose is not None:
#                 self.cur_pose = np.matmul(self.cur_pose, np.linalg.inv(transf))
#             else:
#                 self.cur_pose = np.linalg.inv(transf)
        
#         # Store estimated pose
#         if self.cur_pose is not None:
#             # Convert rotation matrix to Rodrigues vector
#             estimated_rotation, _ = cv2.Rodrigues(self.cur_pose[:3, :3])
            
#             self.estimated_path.append((
#                 estimated_rotation[0, 0],
#                 estimated_rotation[1, 0], 
#                 estimated_rotation[2, 0], 
#                 self.cur_pose[0, 3], 
#                 self.cur_pose[1, 3], 
#                 self.cur_pose[2, 3]
#             ))

#     def get_current_pose(self):
#         """
#         Get the current estimated pose
#         """
#         if self.cur_pose is not None:
#             return self.cur_pose.copy()
#         return None
    
#     def get_estimated_path(self):
#         """
#         Get the full estimated trajectory
#         """
#         return np.array(self.estimated_path) if self.estimated_path else None

