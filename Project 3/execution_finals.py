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
from detect import detect

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
    # targets = [[1, 0.2], [1.5, 0.5], [1, 1], [1.2, 0.2]]
    # targets = [[2.8, 1.1],[1.8, 2.7],[1.8, 3.6],[0.54, 5.3]] # to get from start to enemy closet
    targets = [[2.8, 1.1],[1.8, 2.7],[1.8, 3.6],[0.54, 5.3],[1.8, 3.6],[1.8, 2.7],[2.8,1.1]] # to get from start to enemy closet and back to our closet
    #targets = [[1, 0], [1, -1], [0, -1], [0, 0]] # square

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        targets = robo.sequence(targets)
        #print(targets[0])

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)
        apriltag.draw_detections(img, detections)

        if len(detections) > 0:
            apriltag.movingAvg_tags(detections, robo.globalPose, 5)
        
        # apriltag.troubleshoot() # prints seen tags

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3)

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

CLOSET_ONE = [3, 1.0]
CLOSET_ONE_CORNER1 = [3, 0.8]
CLOSET_TWO = [0.4, 5.2]
CLOSET_TWO_CORNER1 = [0.4, 5.5]
CORRIDOR_ONE = [1.8, 2.6]
INTER_CORRIDOR_ONE = [2.0, 1.8]
CORRIDOR_TWO = [1.8, 3.8]
INTER_CORRIDOR_TWO = [1.6, 4.6]

def merged_brick():
    robo.stow_arm()

    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    graph1, graph2, graph3 = apriltag.initGraph()
    robo.get_robotPosition()
    robo.get_robotAngle()
    firstTime = True
    startTime = time.time()
    ultimatum = 0.5*60

    targets = [CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE, 
               CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO, INTER_CORRIDOR_TWO, CORRIDOR_TWO, CORRIDOR_ONE, INTER_CORRIDOR_ONE]

    # targets = [CLOSET_ONE, 
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
    #            CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE
    #            ]
    
    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        curr_target = targets[0]
        targets = robo.sequence(targets)
        close = None

        print(curr_target)
        print(f'DISTANCE: {((robo.globalPose[0]-CLOSET_TWO[0])**2+(robo.globalPose[1]-CLOSET_TWO[1])**2)**0.5}')

        if curr_target == CLOSET_ONE and ((robo.globalPose[0]-CLOSET_ONE[0])**2+(robo.globalPose[1]-CLOSET_ONE[1])**2)**0.5 < .3:
            # if time.time() - startTime < ultimatum:
            print("CLOSET ONE FIND")
            robo.ready_arm()
            robo.align_with_closet(CLOSET_ONE)
            detector.run_closet_bricks(robo.ep_camera)
            print("CLOSET ONE PICKUP")
            robo.pickup()
            robo.stow_arm()
            detector.backup()
            targets.pop(0)
            if firstTime:
                firstTime = False
            else:
                robo.rotate180()
            # else:
            #     print("THE FINAL COUNT DOWN")
            #     targets.insert(0, CLOSET_ONE_CORNER1)

        if curr_target == CLOSET_TWO and ((robo.globalPose[0]-CLOSET_TWO[0])**2+(robo.globalPose[1]-CLOSET_TWO[1])**2)**0.5 < .3:
            # if time.time() - startTime < ultimatum:
            print("CLOSET TWO RELEASE")
            robo.release()
            time.sleep(1)
            robo.stow_arm()
            detector.backup()
            targets.pop(0)
            robo.rotate180()
            # else:
            #     print("THE FINAL COUNT DOWN")
            #     targets.insert(0, CLOSET_TWO_CORNER1)

        # if ((robo.globalPose[0]-CORRIDOR_ONE[0])**2+(robo.globalPose[1]-CORRIDOR_ONE[1])**2)**0.5 > .2:
        #     robo.avoidTag(close, dist_thres=0.3)
        # elif ((robo.globalPose[0]-CORRIDOR_TWO[0])**2+(robo.globalPose[1]-CORRIDOR_TWO[1])**2)**0.5 > .2:
        #     robo.avoidTag(close, dist_thres=0.3)

        img = cv2.resize(img, (640, 360))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)
        apriltag.draw_detections(img, detections)

        if len(detections) > 0:
            apriltag.movingAvg_tags(detections, robo.globalPose)
            close = apriltag.closestPosition(detections)
            
        # apriltag.troubleshoot()
        # print(robo.vel_y)
        
        # apriltag.troubleshoot() # prints seen tags

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3)            

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

def test_avoid():
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

        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray.astype(np.uint8)
        # detections = apriltag.find_tags(gray)
        # apriltag.draw_detections(img, detections)

        # if len(detections) > 0:
        #     apriltag.movingAvg_tags(detections, robo.globalPose)
        
        # apriltag.troubleshoot() # prints seen tags

        print(f'X: {robo.globalPose[0]}, Y: {robo.globalPose[2]}')

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3)

        condition, coords = robo.avoid_tag(apriltag.relevantTags)
        

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
    robo.stow_arm()
    # robo.ready_arm()
    # robo.release()

def location_test():
    robo.stow_arm()

    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    graph1, graph2, graph3 = apriltag.initGraph()
    robo.get_robotPosition()
    robo.get_robotAngle()

    targets = [CLOSET_ONE, 
               CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
               CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE,
               CORRIDOR_ONE, CORRIDOR_TWO, CLOSET_TWO, CORRIDOR_TWO, CORRIDOR_ONE, CLOSET_ONE
               ]

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        # img = cv2.resize(img, (640, 360))
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray.astype(np.uint8)z
        # detections = apriltag.find_tags(gray)
        # apriltag.draw_detections(img, detections)

        # if len(detections) > 0:
        #     apriltag.movingAvg_tags(detections, robo.globalPose)
        
        # apriltag.troubleshoot() # prints seen tags

        apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3) 

        print(f'X: {robo.globalPose[0]}, Y: {robo.globalPose[1]}')           

        # Display the captured frame
        cv2.imshow('Camera', img)

        if cv2.waitKey(1) == ord('z'):
            break

if __name__ == "__main__":
    # Robot Init
    robo = motion()
    apriltag = AprilTagDetector()
    detector = detect.Detector(robo.ep_chassis)

    try:
        # test_aprilTagGlobal()
        # test_smoothMotion()
        merged_brick()
        # robo.arctan2Test(-0.065, .1)
        # robo.stow_arm()
        # test_avoid()
        # location_test()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        shutdown()