import keyboard
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

def test_aprilTagRelative():
    '''
    Get relative positioning of AprilTags and represent them graphically
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    graph, quiver = apriltag.initGraph()
    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        robo.teleop()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)

        if len(detections) > 0:
            apriltag.plot_detections(detections, graph, quiver)

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

def test_aprilTagGlobal():
    '''
    Get global positioning of AprilTags and represent them graphically
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    graph1, graph2, graph3 = apriltag.initGraph()
    robo.get_robotPosition()
    robo.get_robotAngle()

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        robo.teleop()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)
        apriltag.draw_detections(img, detections)

        if len(detections) > 0:
            apriltag.refine_tags(detections, robo.globalPose, 0.1)
            apriltag.troubleshoot()

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3)
        print(robo.globalPose[2])

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

def test_smoothMotion():
    '''
    Get global positioning of AprilTags and represent them graphically
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    graph1, graph2, graph3 = apriltag.initGraph()
    robo.get_robotPosition()
    robo.get_robotAngle()
    targets = [[2.8, 1.1],[1.8, 2.7],[1.8, 3.6],[0.54, 5.3]]
    # targets = [[1, 0], [1, -1], [0, -1], [0, 0]]

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        targets = robo.sequence(targets)
        # print(targets)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)
        apriltag.draw_detections(img, detections)

        if len(detections) > 0:
            apriltag.refine_tags(detections, robo.globalPose, 0.15)
        
        apriltag.troubleshoot()

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3)

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

def show_camera_feed():
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('q'):
            break

def doarmstuff():
    # robo.get_gripper_status()
    robo.ready_arm()
    robo.release()

if __name__ == "__main__":
    # Robot Init
    robo = motion()
    apriltag = AprilTagDetector()

    try:
        # test_aprilTagGlobal()
        test_smoothMotion()
        # robo.arctan2Test(-0.065, .1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        shutdown()