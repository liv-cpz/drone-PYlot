from djitellopy import Tello
import cv2
import time

tello = Tello()

tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()

# time.sleep(1)
time.sleep(0.5)
cv2.imwrite("tello-imm.png", frame_read.frame)

tello.streamoff()

# tello.takeoff()

# tello.move_left(100)
# tello.rotate_clockwise(90)
# tello.move_forward(100)

# tello.land()