from djitellopy import Tello
import time
from tello_vio import VIO
import numpy as np
from scipy.spatial.transform import Rotation as R

tello = Tello()

tello.connect()

tello_dict = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "vgx":0,
    "vgy":0,
    "vgz":0,
    "agx":0,
    "agy":0,
    "agz":0
}


vio = VIO()

t = np.array([0,0,0]).T
tello_dict["roll"] = tello.get_roll()
tello_dict["pitch"] = tello.get_pitch()
tello_dict["yaw"] = tello.get_yaw()
# check zyx = pitch roll yaw? intrinsic or extrinsic?
r = R.from_euler('zyx', [tello_dict["pitch"], tello_dict["roll"], tello_dict["yaw"] ], degrees=True)

vio.set_initial_pose(r, t)
tello.streamon()

frame = tello.get_frame_read()
vio.set_initial_image(frame.frame)
vio.set_initial_imu(tello_dict)


for i in range(20):
    # note this info is NOT coming from imu? 
    # therefore can filter with accelerom data

    tello_dict["agx"] = tello.get_acceleration_x()/100
    tello_dict["agy"] = tello.get_acceleration_y()/100
    tello_dict["agz"] = tello.get_acceleration_z()/100

    print(tello_dict)
    frame = tello.get_frame_read()
    image = frame.frame

    time.sleep(0.1)
    vio.run(image, tello_dict)

    print(vio.estimated_path[-1])
    time.sleep(0.5)


tello.streamoff()