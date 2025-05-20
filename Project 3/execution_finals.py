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
import os

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

CLOSET_ONE = [2.9, 1]
CLOSET_ONE_CORNER1 = [3, 0.8]
CLOSET_TWO = [0.8, 5.2]
CLOSET_TWO_CORNER1 = [0.4, 5.5]
CORRIDOR_ONE = [1.8, 2.6]
INTER_CORRIDOR_ONE = [2.0, 1.95]
INTER2_CORRIDOR_ONE = [2.0, 1.1]
CORRIDOR_TWO = [2.0, 3.8]
INTER_CORRIDOR_TWO = [1.6, 4.45]

def merged_brick():
    robo.stow_arm()

    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    graph1, graph2, graph3, graph4 = apriltag.initGraph()
    robo.get_robotPosition()
    robo.get_robotAngle()
    firstTime = True
    pathComplete = False
    avoidTriggered = False
    targetPointer = 0

    defaultTargets = [CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO]
    targets = [CLOSET_ONE, INTER_CORRIDOR_ONE, CORRIDOR_ONE, CORRIDOR_TWO, INTER_CORRIDOR_TWO, CLOSET_TWO]

    
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
    file = open_new_file("..\\data")
    with open(file, "w") as data:
        while True:
            try:
                img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            except Empty:
                time.sleep(0.001)
                continue
            
            curr_target = targets[targetPointer]
            closeTag = None

            # Make robot "go-to" a position, if it hasn't reached target yet it will stay false and not change the pointer
            if robo.sequence(curr_target):
                targetPointer += 1
                if curr_target in defaultTargets:
                    avoidTriggered = False

            if curr_target == CLOSET_ONE and ((robo.globalPose[0]-CLOSET_ONE[0])**2+(robo.globalPose[1]-CLOSET_ONE[1])**2)**0.5 < 0.15:
                print("CLOSET ONE FIND")
                robo.ready_arm()
                robo.align_with_closet(CLOSET_ONE)
                detector.run_closet_bricks(robo.ep_camera)
                print("CLOSET ONE PICKUP")
                robo.pickup()
                robo.stow_arm()
                detector.backup()
                
                if firstTime:
                    firstTime = False
                    targets.pop(0)
                    targetPointer = 1
                else:
                    targets = targets[::-1]
                    robo.rotate180()
                    targetPointer = 1

            if curr_target == CLOSET_TWO and ((robo.globalPose[0]-CLOSET_TWO[0])**2+(robo.globalPose[1]-CLOSET_TWO[1])**2)**0.5 < 0.15:
                print("CLOSET TWO RELEASE")
                robo.release()
                time.sleep(1)
                robo.stow_arm()
                detector.backup()
                targetPointer = 1
                targets = targets[::-1]
                robo.rotate180()
                if not pathComplete:
                    pathComplete = True

            img = cv2.resize(img, (640, 360))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)
            detections = apriltag.find_tags(gray)
            apriltag.draw_detections(img, detections)

            if len(detections) > 0:
                apriltag.movingAvg_tags(detections, robo.globalPose)

                closeTag = apriltag.closest(detections)
                closeTagPos = apriltag.return_global_position(closeTag)
                closeTagPos_rel = apriltag.get_box_relative(closeTag)
                if curr_target != CORRIDOR_ONE or curr_target != CORRIDOR_TWO:
                    dist = closeTagPos_rel[0]**2 + closeTagPos_rel[1]**2
                    if dist < 0.85 and abs(closeTagPos_rel[0]) < 0.27 and not pathComplete and not avoidTriggered and closeTagPos != None:
                        destination = robo.avoidTag(closeTagPos, offset=0.6)
                        targets.insert(targetPointer, destination)
                        avoidTriggered = True

            if closeTagPos == None:
                closeTagPos = [0, 0]
            
            save_data(data, f"{robo.globalPose[0]}, {robo.globalPose[1]}, {curr_target[0]}, {curr_target[1]}, {closeTagPos[0]}, {closeTagPos[1]}\n")

            # apriltag.troubleshoot() # prints seen tags

            apriltag.plot_detections(robo.globalPose, graph1, graph2, graph3, graph4, curr_target)            
            # print(f"Allowed to Retarget: {not(avoidTriggered)}, Current Target: {curr_target}, Side Distance from Box: {closeTagPos_rel[0]}")
            # print(f'DISTANCE: {((robo.globalPose[0]-CLOSET_TWO[0])**2+(robo.globalPose[1]-CLOSET_TWO[1])**2)**0.5}')

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

def open_new_file(filepath):
    '''
    Saves content to a new file, avoiding overwriting existing files.

    Args:
        filepath: The desired file path (including filename).
        content: The content to write to the file.
    '''
    
    _, ext = os.path.splitext(filepath)
    count = 1
    while os.path.exists(filepath):
        filepath = f"data_{count}{ext}"
        count += 1
    
    return filepath

def save_data(file, content):
    try:
        file.write(content)
    except Exception as e:
        print(f"An error occurred: {e}")

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