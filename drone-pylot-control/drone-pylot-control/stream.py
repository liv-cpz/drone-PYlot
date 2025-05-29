import time, cv2
from threading import Thread
from djitellopy import Tello
# note --> need sleep 

tello = Tello()

tello.connect()

keepRecording = True
tello.streamon()

def videoRecorder():
    frame_read = tello.get_frame_read()
    time.sleep(0.2)
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video_tello.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 2)
        frame_read = tello.get_frame_read()

    video.release()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=videoRecorder)
recorder.start()

time.sleep(5)

keepRecording = False
tello.streamoff()
recorder.join()