from djitellopy import Tello, TelloException
import cv2
import numpy as np
import time 
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
from simple_pid import PID
import logging

import signal
import sys
import os

# from scipy.spatial.transform import Rotation
# import scipy.signal
from pyr_lucas_kanade import lucas_pyramidal


# configure logging
target_format = '%(asctime)s [%(levelname)s] %(message)s'
logging.basicConfig(format=target_format, level=logging.INFO)
logger = logging.getLogger(__name__)


def signal_handler(sig, frame):
    """Signal handler to gracefully exit on Ctrl+C."""
    print("Exiting...")
    track_tello.mytello.land()
    stream = False
    track_tello.mytello.streamoff()
    track_tello.mytello.end()
    sys.exit(0)

def load_ost_calibration(path: str):
    """Parse oST-format text and return camera matrix and distortion coefficients."""
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]
    K = dist = None
    in_narrow = False
    i = 0
    while i < len(lines):
        l = lines[i].lower()
        if l == '[narrow_stereo]':
            in_narrow = True
            i += 1
            continue
        if in_narrow:
            if l == 'camera matrix':
                rows = [list(map(float, lines[i + j].split())) for j in (1, 2, 3)]
                K = np.array(rows, dtype=float)
                i += 4
                continue
            if l == 'distortion':
                dist = np.array(list(map(float, lines[i + 1].split())), dtype=float)
                i += 2
                continue
            if l.startswith('['):
                break
        i += 1
    if K is None or dist is None:
        raise RuntimeError(f'Failed to parse calibration from {path}')
    return K, dist


@dataclass
class PIDConfig:
    """@dataclass for PID configuration parameters.
    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (float): Desired setpoint for the PID controller.
        output_limits (Tuple[float, float]): Limits for the output of the PID controller.
        sample_time (Optional[float]): Time interval for PID updates, if None, uses default.
    """
    kp: float
    ki: float
    kd: float
    setpoint: float
    output_limits: Tuple[float, float] = (-20, 20) # Default output limits
    sample_time: Optional[float] = None
    
    

class DronePIDController:
    """DronePIDController implements a PID controller for drone navigation.
    It uses PID controllers for x, y, z, and yaw axes to compute velocity commands.   
    It takes a configuration dictionary to initialize the PID controllers.
    Attributes:
        pid_x (PID): PID controller for x-axis velocity.   
        pid_y (PID): PID controller for y-axis velocity.
        pid_z (PID): PID controller for z-axis velocity.
        pid_yaw (PID): PID controller for yaw velocity.
    """  
    pid_x: PID
    pid_y: PID
    pid_z: PID
    pid_yaw: PID

    def __init__(self, config: Dict[str, PIDConfig]):
        self.pid_x = PID(config['x'].kp, config['x'].ki, config['x'].kd,
                         setpoint=config['x'].setpoint,
                         output_limits=config['x'].output_limits,
                         sample_time=config['x'].sample_time)
        self.pid_y = PID(config['y'].kp, config['y'].ki, config['y'].kd,
                         setpoint=config['y'].setpoint,
                         output_limits=config['y'].output_limits,
                         sample_time=config['y'].sample_time)
        self.pid_z = PID(config['z'].kp, config['z'].ki, config['z'].kd,
                         setpoint=config['z'].setpoint,
                         output_limits=config['z'].output_limits,
                         sample_time=config['z'].sample_time)
        self.pid_yaw = PID(config['yaw'].kp, config['yaw'].ki, config['yaw'].kd,
                           setpoint=config['yaw'].setpoint,
                           output_limits=config['yaw'].output_limits,
                           sample_time=config['yaw'].sample_time)
        
        self.pid_x2 = PID(config['x2'].kp, config['x2'].ki, config['x2'].kd,
                         setpoint=config['x2'].setpoint,
                         output_limits=config['x2'].output_limits,
                         sample_time=config['x2'].sample_time)
        self.pid_y2 = PID(config['y2'].kp, config['y2'].ki, config['y2'].kd,
                         setpoint=config['y'].setpoint,
                         output_limits=config['y2'].output_limits,
                         sample_time=config['y2'].sample_time)
        self.pid_z2 = PID(config['z2'].kp, config['z2'].ki, config['z2'].kd,
                         setpoint=config['z2'].setpoint,
                         output_limits=config['z2'].output_limits,
                         sample_time=config['z2'].sample_time)
        
        
        logger.info("Drone PID controller initialised")
        
        
    def update(self, target_center:Tuple[float, float], 
               target_area:float,
               image_width:float,
               image_height:float,
               d_yaw:float = 0.0) -> Tuple[float, float, float, float]:
        """
        Update the PID controllers with the current and target poses.
        Args:
            current_pose (Tuple[float, float, float, float]): Current pose of the drone (x, y, z, yaw).
            target_pose (Tuple[float, float, float, float]): Target pose for the drone (x, y, z, yaw).
        Returns:
            Tuple[float, float, float, float]: Velocity commands for x, y, z, and yaw.
        """
        desired_area = 20000

        if not target_center or not target_area or not image_width:
            return 0.0, 0.0, 0.0, 0.0
        
        x_error = image_width/2 - target_center[0] 
        y_error = target_area-desired_area
        z_error = target_center[1] -image_height/4
        yaw_error = d_yaw
        
        vx = self.pid_x(x_error)
        vy = self.pid_y(y_error)
        vz = self.pid_z(z_error)
        vyaw = self.pid_yaw(yaw_error)
        return vx, vy, vz, vyaw
    
    def update_aruco(self, Tv: np.ndarray) -> Tuple[float, float, float, float]:
        """Update the PID controllers with ArUco marker poses.
        Args:
            Tv (np.ndarray): Camera to vertical marker pose.
            Th (np.ndarray): Camera to horizontal marker pose.
        Returns:
            Tuple[float, float, float, float]: Velocity commands for x, y, z, and yaw.
        """
        if Tv is None:
            return 0.0, 0.0, 0.0, 0.0
        
        # extract the camera→horizontal translation
        x_cam, y_cam, z_cam = Tv[:3, 3]

         #x=-0.018, y=-0.152, z=0.569
        x_err= -x_cam
        y_err= y_cam+0.1
        z_err= -z_cam

        # map into drone’s own axes
        dx =  x_err*100       # positive → roll right
        dy = z_err*100       # positive → pitch forward
        dz = y_err*100      # positive → throttle up

        # logger.info(f"dx: {dx}, dy: {dy}, dz: {dz}")
        # logger.info(f"dy:{dz}, z_cam:{y_ca/m}")
        vx = self.pid_x2(dx)
        vy = self.pid_y2(dy)
        vz = self.pid_z2(dz)
        vyaw = 0
        return vx, vy, vz, vyaw
    
    
    def reset_pid(self):
        """Reset PID controllers to initial state.
        """
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()

        self.pid_x2.reset()
        self.pid_y2.reset()
        self.pid_z2.reset()
        self.pid_yaw.reset()


# class VisualOdometry():
#     def __init__(self):
#         self.K, self.P = self._load_calib("calib.txt")

#         self.R_cam_to_world = np.array([
#             [1,  0,  0],   # x_cam → x_world
#             [0, -1,  0],   # y_cam → -y_world (flip Y for display)
#             [0,  0, -1],   # z_cam → -z_world (flip Z for display)
#         ])

#         self.image_current = None
#         self.image_prev = None

#     @staticmethod
#     def _load_calib(filepath):
#         with open(filepath, 'r') as f:
#             params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
#             P = np.reshape(params, (3, 4))
#             K = P[0:3, 0:3]
#         return K, P

#     @staticmethod
#     def _form_transf(R, t):
#         T = np.eye(4, dtype=np.float64)
#         T[:3, :3] = R
#         T[:3, 3] = t
#         return T
    
#     def set_initial_image(self, image):
#         self.prev_image = image
    
#     def get_matches(self, img1, img2, number_features=200, wz=5, level=5, number_iteration=70, inlier_threshold=3, static=False):
#         q1, q2, _ = lucas_pyramidal(img1, img2, number_features, wz, level, number_iteration, inlier_threshold, static)
#         return q1, q2
    
#     def get_pose(self, q1, q2):
#         E, _ = cv2.findEssentialMat(q1, q2, self.K, method=0, threshold=0.1)
#         R, t = self.decomp_essential_mat(E, q1, q2)
#         return R, t
    
#     def decomp_essential_mat(self, E, q1, q2):
#         def sum_z_cal_relative_scale(R, t):
#             T = self._form_transf(R, t)
#             P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
#             hom_Q1 = cv2.triangulatePoints(np.float32(self.P), np.float32(P), np.float32(q1.T), np.float32(q2.T))
#             hom_Q2 = np.matmul(T, hom_Q1)
#             uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
#             uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
#             return sum(uhom_Q1[2, :] > 0) + sum(uhom_Q2[2, :] > 0)

#         R1, R2, t = cv2.decomposeEssentialMat(E)
#         t = np.squeeze(t)
#         pairs = [[R1, -t], [R1, t], [R2, t], [R2, -t]]
#         z_sums = [sum_z_cal_relative_scale(R, t) for R, t in pairs]
#         right_pair_idx = np.argmax(z_sums)
#         return pairs[right_pair_idx]

#     def run(self, image):
#         if self.image_current is None:
#             self.image_current = image
#             self.set_initial_image(image)
#             return None, None
        
#         q1, q2 = self.get_matches(self.image_prev, image)
#         R, t = self.get_pose(q1, q2)
        
#         # Update the current image
#         self.image_prev = self.image_current
#         self.image_current = image
        
#         # Transform the rotation to world coordinates
#         R_world = np.matmul(self.R_cam_to_world, R) #estimate of the rotation
#         t_world = np.matmul(self.R_cam_to_world, t)
        
#         return R_world, t_world



# === ArUco Tracker ===

class ArucoTracker:
    def __init__(self, K, dist, tag_length, vertical_id, horizontal_id,
                 dict_id=cv2.aruco.DICT_6X6_50):
        self.K = K
        self.dist = dist
        self.tag_len = tag_length
        self.vid = vertical_id
        self.hid = horizontal_id
        # marker detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        # relative transform and smoothing state
        self.T_h_to_v = None
        self.smoothed_Tv = None
        self.alpha = 0.2  # smoothing factor [0,1]

    def detect(self, frame):
        """Detect markers and compute each tag's 4×4 pose via solvePnP."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)
        poses = {}
        if ids is None:
            return poses
        ids = ids.flatten()
        # define 3D object points of the four tag corners
        s = self.tag_len / 2.0
        objp = np.array([[-s,  s, 0], [ s,  s, 0], [ s, -s, 0], [-s, -s, 0]], dtype=float)
        # for each detected marker
        for idx, tag_id in enumerate(ids):
            corners = corners_list[idx].reshape(-1, 2)
            # solvePnP to get rotation+translation
            # drop flags arg to use default iterative solver
            retval, rvec, tvec = cv2.solvePnP(objp, corners, self.K, self.dist)
            if not retval:
                continue
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            poses[tag_id] = T
        return poses

    def update(self, frame):
        """Return smoothed camera→vertical and camera→horizontal poses, update relative transform."""
        poses = self.detect(frame)
        Tv = poses.get(self.vid)
        Th = poses.get(self.hid)
        # smoothing Tv translation
        if Tv is not None:
            t = Tv[:3, 3]
            if self.smoothed_Tv is None:
                self.smoothed_Tv = t.copy()
            else:
                self.smoothed_Tv = self.alpha * t + (1 - self.alpha) * self.smoothed_Tv
            Tv = Tv.copy()
            Tv[:3, 3] = self.smoothed_Tv
        # update relative when both seen
        if Tv is not None and Th is not None:
            self.T_h_to_v = np.linalg.inv(Tv) @ Th
        # estimate missing horizontal
        if Th is None and Tv is not None and self.T_h_to_v is not None:
            Th = Tv @ self.T_h_to_v
        return Tv, Th




# This code is a simple drone controller that uses computer vision to track a colored object

class TelloTracker:
    def __init__(self, width=640, height=480, dead_zone=100):
        self.width = width
        self.height = height
        self.dead_zone = dead_zone
        self.mytello = Tello()
        self.mytello.connect()
        
        print(f"Battery: {self.mytello.get_battery()}%")
        self.mytello.streamoff()
        self.mytello.streamon()

        self.tag_track = False
        
        self.startCounter = 0

        config = {
            'x': PIDConfig(kp=0.6, ki=0.2, kd=0.3, setpoint=0.0, output_limits=(-15, 15)),
            'y': PIDConfig(kp=2.5, ki=0.1, kd=1.2, setpoint=0.0, output_limits=(-20, 20)),
            'z': PIDConfig(kp=1.2, ki=0.0, kd=0.15, setpoint=0.0, output_limits=(-15, 15)),  # z is fine for now
            'yaw': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=0.0),   # yaw is fine
            'x2': PIDConfig(kp=0.6, ki=0.2, kd=0.3, setpoint=0.0, output_limits=(-10, 10)),
            'y2': PIDConfig(kp=2.5, ki=0.1, kd=1.2, setpoint=0.0, output_limits=(-20, 20)),
            'z2': PIDConfig(kp=1.2, ki=0.0, kd=0.15, setpoint=0.0, output_limits=(-10, 10)),  # z is fine for now
        }

        self.drone_controller = DronePIDController(config)

        self.aruco_tracker = None
        self.init_aruco_tracker()

        # self.vo = VisualOdsometry()

    def init_aruco_tracker(self):
        """Initialize the ArUco tracker with camera calibration parameters."""
        base = os.path.dirname(os.path.abspath(__file__))
        calib = os.path.normpath(os.path.join(base, '..', 'aruco_det', 'ost.txt'))
        K, dist = load_ost_calibration(calib)
        tag_length = 0.1
        vertical_id = 23
        horizontal_id = 24
        self.aruco_tracker = ArucoTracker(K, dist, tag_length, vertical_id, horizontal_id)

    def run(self):
        """Main loop to run the drone tracking."""
        stream = True
        idx = 0
        pts = [(0, 0)] * 10
        ctr = 0

        while stream:
            time.sleep(0.2)
            myFrame =  self.mytello.get_frame_read().frame
            fr = myFrame.copy()
            
            Tv, Th = self.aruco_tracker.update(fr)

            if Tv is not None and Th is not None and self.tag_track == False:
                # set flag
                self.tag_track = True
                print("")
                print("GOTTEM")
                print("")
                self.drone_controller.reset_pid()
                continue

            if not self.tag_track:
                img = cv2.resize(fr, (self.width, self.height))
                blur = cv2.GaussianBlur(img, (5, 5), 0)
                hsv_img = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

                lower = np.array([36, 50, 50])
                upper = np.array([86, 255, 255])

                mask = cv2.inRange(hsv_img, lower, upper)
                kernel = np.ones((7, 7), np.uint8)

                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                segmented = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)
                h, s, v = cv2.split(segmented)

                cnts, _ = cv2.findContours(h,
                                        cv2.RETR_TREE,
                                        cv2.CHAIN_APPROX_NONE)
                
                # output = cv2.drawContours(img, cnts, -1, (0, 0, 255), 3)


                center = None

                # only proceed if at least one contour was found
                if len(cnts) > 0:
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    area = cv2.contourArea(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    if radius > 10:
                        cv2.circle(img, (int(x), int(y)), int(radius),
                                    (0, 255, 255), 2)
                        cv2.circle(img, center, 5, (0, 0, 255), -1)
                    pts[ctr % 10] = center
                    ctr += 1

                    # loop over the set of tracked points
                    for i in range(1, len(pts)):
                            # if either of the tracked points are None, ignore
                            # them
                            if pts[i - 1] is None or pts[i] is None:
                                    continue
                            cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), 1)


                    # R_world, t_world = self.vo.run(img)

                    # Rot_mat = Rotation.from_matrix(R_world)
                    # euler_angles = Rot_mat.as_euler('xyz', degrees=True)
                    # show the frame to our screen
                    print("calculating..")
                    vx, vy, vz, vyaw = self.drone_controller.update(center, area, self.width, self.height, d_yaw=0.0)
                    
                    print(f"dx: {vx}, dy: {vy}, dz: {vz}, vyaw: {vyaw}")
                    self.mytello.send_rc_control(int(vx), int(vy), int(vz), 0)
                    cv2.imwrite(f"POV{idx}.png", img)
            else:

                if Tv is None and Th is None:
                    self.mytello.land()
                    self.mytello.streamoff()
                    self.mytello.end()

                if Tv is not None:
                    # then call your PID controller:
                    vx, vy, vz, vyaw = self.drone_controller.update_aruco(Tv)

                    self.mytello.send_rc_control(int(vx), int(vy), int(vz), 0)

                # absolute vertical
                if Tv is not None:
                    pv = Tv[:3, 3]
                    logger.info(f'Vertical @ x={pv[0]:.3f}, y={pv[1]:.3f}, z={pv[2]:.3f}')
                else:
                    logger.info('Vertical not detected')

                # absolute horizontal
                if Th is not None:
                    ph = Th[:3, 3]
                    logger.info(f'Horizontal @ x={ph[0]:.3f}, y={ph[1]:.3f}, z={ph[2]:.3f}')
                else:
                    logger.info('Horizontal not detected')

                # relative while both visible
                if Tv is not None and Th is not None and self.aruco_tracker.T_h_to_v is not None:
                    rel = self.aruco_tracker.T_h_to_v
                    rv = rel[:3, 3]
                    logger.info(f'Relative V→H offset x={rv[0]:.3f}, y={rv[1]:.3f}, z={rv[2]:.3f}')
                # estimate when horizontal missing
                elif Th is None and Tv is not None and self.aruco_tracker.T_h_to_v is not None:
                    Th_est = Tv @ self.aruco_tracker.T_h_to_v
                    eh = Th_est[:3, 3]
                    logger.info(f'Estimated H from V x={eh[0]:.3f}, y={eh[1]:.3f}, z={eh[2]:.3f}')


            if self.startCounter == 0:
                self.mytello.takeoff()
                self.startCounter = 1

            # cv2.imwrite(f"mask_{idx}.jpg", output)
            idx += 1



track_tello = TelloTracker()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    # track_tello = TelloTracker()

    track_tello.run()



if __name__ == "__main__":
    main()