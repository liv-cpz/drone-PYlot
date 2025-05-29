# aruco_tracker.py

import os
import sys
import time
import signal
import logging

import cv2
import numpy as np
from djitellopy import Tello, TelloException

# configure logging
target_format = '%(asctime)s [%(levelname)s] %(message)s'
logging.basicConfig(format=target_format, level=logging.INFO)
logger = logging.getLogger(__name__)

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

# === Clean exit support ===

running = True
signal.signal(signal.SIGINT, lambda s, f: globals().update(running=False))

# === Main loop ===

if __name__ == '__main__':
    # load camera calibration
    base = os.path.dirname(os.path.abspath(__file__))
    calib = os.path.normpath(os.path.join(base, '..', 'aruco_det', 'ost.txt'))
    K, dist = load_ost_calibration(calib)
    tracker = ArucoTracker(K, dist, tag_length=0.1, vertical_id=23, horizontal_id=24)

    # try Tello video stream, else webcam
    use_webcam = False
    missing = 0
    logger.info('Connecting to Tello…')
    tello = Tello()
    tello.connect()
    tello.streamoff(); tello.streamon()
    reader = tello.get_frame_read()
    time.sleep(2.0)
    logger.info(f'Tello connected, battery {tello.get_battery()}%')
    logger.info('Starting detection loop; press Ctrl+C to exit')
    while running:
        # grab frame
        if use_webcam:
            ret, frame = reader.read()
            if not ret:
                time.sleep(0.1)
                continue
        else:
            frame = reader.frame
            if frame is None:
                missing += 1
                if missing > 30:
                    logger.warning('Tello frames lost, switching to webcam')
                    reader = cv2.VideoCapture(0)
                    use_webcam = True
                    missing = 0
                time.sleep(0.1)
                continue
            missing = 0

        # update poses
        Tv, Th = tracker.update(frame)

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
        if Tv is not None and Th is not None and tracker.T_h_to_v is not None:
            rel = tracker.T_h_to_v
            rv = rel[:3, 3]
            logger.info(f'Relative V→H offset x={rv[0]:.3f}, y={rv[1]:.3f}, z={rv[2]:.3f}')
        # estimate when horizontal missing
        elif Th is None and Tv is not None and tracker.T_h_to_v is not None:
            Th_est = Tv @ tracker.T_h_to_v
            eh = Th_est[:3, 3]
            logger.info(f'Estimated H from V x={eh[0]:.3f}, y={eh[1]:.3f}, z={eh[2]:.3f}')

        time.sleep(0.2)

    # cleanup
    if use_webcam:
        reader.release()
    else:
        tello.streamoff(); tello.end()
    logger.info('Exited cleanly')
    sys.exit(0)