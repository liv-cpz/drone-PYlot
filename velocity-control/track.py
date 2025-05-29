from djitellopy import Tello
import cv2
import numpy as np
import time 
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
from simple_pid import PID
import logging

from threading import Thread
import signal
import sys
import pygame

from scipy.spatial.transform import Rotation
import scipy.signal
from pyr_lucas_kanade import lucas_pyramidal



logger = logging.getLogger(__name__)

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
        
        
        logger.info("Drone PID controller initialised")
        
        
    def update(self, target_center:Tuple[float, float], 
               target_area:float,
               image_width:float,
               image_height:float) -> Tuple[float, float, float, float]:
        """
        Update the PID controllers with the current and target poses.
        Args:
            current_pose (Tuple[float, float, float, float]): Current pose of the drone (x, y, z, yaw).
            target_pose (Tuple[float, float, float, float]): Target pose for the drone (x, y, z, yaw).
        Returns:
            Tuple[float, float, float, float]: Velocity commands for x, y, z, and yaw.
        """
        desired_area = 8200

        if not target_center or not target_area or not image_width:
            return 0.0, 0.0, 0.0, 0.0
        
        x_error = image_width/2 - center[0] 
        y_error = target_area-desired_area
        z_error = center[1] - image_height/2
        yaw_error = 0.0
        
        vx = self.pid_x(x_error)
        vy = self.pid_y(y_error)
        vz = self.pid_z(z_error)
        vyaw = self.pid_yaw(yaw_error)
        return vx, vy, vz, vyaw
    
    
    def reset_pid(self):
        """Reset PID controllers to initial state.
        """
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()


class VisualOdometryIMU():
    def __init__(self):
        self.K, self.P = self._load_calib("calib.txt")

        self.R_cam_to_world = np.array([
            [1,  0,  0],   # x_cam → x_world
            [0, -1,  0],   # y_cam → -y_world (flip Y for display)
            [0,  0, -1],   # z_cam → -z_world (flip Z for display)
        ])

        self.image_current = None
        self.image_prev = None

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
    
    def set_initial_image(self, image):
        self.prev_image = image
    
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





def signal_handler(sig, frame):
    """Signal handler to gracefully exit on Ctrl+C."""
    print("Exiting...")
    mytello.land()
    stream = False
    mytello.streamoff()
    sys.exit(0)


def videoRecorder():

    # Idk but this is needed
    time.sleep(1)

    # Initialise display
    myFrame = frame_read.frame
    height, width = myFrame.shape[:2]
    screen = pygame.display.set_mode((width, height))

    # Loop until 
    while stream:

        # Convert image to a format readable by pygame
        myFrame = frame_read.frame
        frame_surface = pygame.surfarray.make_surface(myFrame.swapaxes(0, 1))

        # Render image
        screen.blit(frame_surface, (0, 0))
        pygame.display.update()

        # Wait until next frame is updated
        time.sleep(1/30)

    pygame.quit()


# Todo fix this code its super messy


######################################################################
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
deadZone =100
######################################################################

signal.signal(signal.SIGINT, signal_handler)
startCounter =0

stream = True

# CONNECT TO TELLO
mytello = Tello()
mytello.connect()



print(mytello.get_battery())

mytello.streamoff()
mytello.streamon()
######################## 

frameWidth = width
frameHeight = height

frame_read = mytello.get_frame_read()
myFrame = frame_read.frame


config = {
    'x': PIDConfig(kp=0.6, ki=0.2, kd=0.3, setpoint=0.0, output_limits=(-10, 10)),
    'y': PIDConfig(kp=2.5, ki=0.1, kd=1.2, setpoint=0.0, output_limits=(-20, 20)),
    'z': PIDConfig(kp=1.2, ki=0.0, kd=0.15, setpoint=1.0, output_limits=(-20, 20)),  # z is fine for now
    'yaw': PIDConfig(kp=1.0, ki=0.0, kd=0.1, setpoint=0.0)   # yaw is fine
}

drone_controller = DronePIDController(config)


# cap   = cv2.VideoCapture(0)s
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
deadZone =100
pts = [(0,0)]*10
ctr = 0

idx = 0

# recorder = Thread(target=videoRecorder, args=())
# recorder.start()

# Initialize the Tello drone
# drone = DronePIDController()
while True:
    # success, myFrame = cap.read()
    time.sleep(0.2)
    myFrame =frame_read.frame
    fr = myFrame.copy()
    
    img = cv2.resize(fr, (width, height))
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv_img = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower = np.array([36, 50, 50])
    upper = np.array([86, 255, 255])

    mask = cv2.inRange(hsv_img, lower, upper)
    kernel = np.ones((7, 7), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    segmented = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)
    h,s,v = cv2.split(segmented)


    cnts, hierarchy = cv2.findContours(h,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_NONE)
    
    output = cv2.drawContours(img, cnts, -1, (0,0,255), 3)

    center = None

    print("setup ok")
    print(len(cnts))
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(img, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                cv2.circle(img, center, 5, (0, 0, 255), -1)
        pts[ctr%10] = center
        ctr = ctr+1

    # loop over the set of tracked points
        for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                        continue
                cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), 1)

        # show the frame to our screen
        print("calculating..")
        print(area)
        vx, vy, vz, vyaw = drone_controller.update(center, area, width, height)
        print(f"dx: {vx}, dy: {vy}, dz: {vz}, vyaw: {vyaw}")
        mytello.send_rc_control(0, 0, int(vz), 0)
    if startCounter == 0:
        mytello.takeoff()
        startCounter = 1

    # cv2.imwrite(f"mask_{idx}.jpg", output)
    idx += 1
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        mytello.land()
        mytello.streamoff()
        break

    if idx > 300:
        mytello.land()
        mytello.streamoff()
        stream = False
        recorder.join()
        break