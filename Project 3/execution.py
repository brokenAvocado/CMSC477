import pupil_apriltags
import cv2
import numpy as np
import time
import traceback
from queue import Empty
import robomaster
from robomaster import robot
from robomaster import camera
import threading

from motion import motion
from apriltag import AprilTagDetector

def shutdown():
    '''
    Sequence for stopping the robot 
    '''
    robo.ep_arm.unsub_position()
    print('Waiting for robomaster shutdown')
    robo.ep_camera.stop_video_stream()
    robo.ep_robot.close()

def get_position(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray.astype(np.uint8)
    detections = apriltag.find_tags(gray)

    if len(detections) > 0:
        detection = apriltag.closest(detections)
        pos, rot = apriltag.get_pose_camera_frame(detection)
        x = pos[2]
        y = pos[0]
        print(f"Position X: {x}, Position Y: {y}, Rotation: {rot}")

    apriltag.draw_detections(frame, detections)

def apriltag_test():
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        get_position(img)
        thread1 = threading.Thread(target=robo.orbit)
        thread1.start()

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == "__main__":
    # Robot Init
    robo = motion()
    apriltag = AprilTagDetector()

    try:
        apriltag_test()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        shutdown()