import pupil_apriltags
import cv2
import numpy as np
import time
import traceback
from queue import Empty
import time
import robomaster
from robomaster import robot
from robomaster import camera

from motion import motion
from motion import AprilTagDetector
from detect import Detect
import threading

# Perception Functions

def updateImage(detector_class, img):
    '''
    Used to update the images from the detector with masks and edges
    Returns the masked brick object
    '''
    mask = detector_class.detect_object(img, detector_class.RED)
    edges = detector_class.edges(mask)
    center = detector_class.center(edges)
    brick = detector_class.isolate_brick(edges)

    return brick, center

def updateDistances(detector_class, img, center_obj):
    '''
    Gets the distance of block from centroid 
    Returns the positions (x = pixels, y = meters)
    '''
    # Get the pixel that the brick centroid is
    center_x, center_y = center_obj
    pos_x = center_x-detector_class.FRAME_CENTER_X
    # Get distance to brick
    pos_y = detector_class.distance_area_far(img)

    return (pos_x, pos_y)

# System Functions:

def shutdown(robot_class):
    '''
    Sequence for stopping the robot 
    '''
    robot_class.ep_arm.unsub_position()
    print('Waiting for robomaster shutdown')
    robot_class.ep_camera.stop_video_stream()
    robot_class.ep_robot.close()

def initTest(robot_class):
    '''
    Initialize our required class for detection and variables for detection and movement
    '''

    # Detector Init
    detector_class = Detect()
    robot_class.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    # Initialize our variables
    errorX = 10 # Just so we don't trigger the conditional early, set arbitrarily to 10
    errorY = 10
    isAligned = True

    return detector_class, errorX, errorY, isAligned

# Movement Functions

def canOrbit(x, y, alignment, x_tol=5, y_tol=0.05):
    '''
    Checks for the conditions to orbit around the object
    '''
    if abs(x) < x_tol and abs(y) < y_tol and alignment:
        return True
    else:
        return False
    
def move_approach(robot_class, detector_class, brickMask, center_obj, errorX, errorY):
    pos = updateDistances(detector_class, brickMask, center_obj)
    isAligned = detector_class.orientation_avg(brickMask)

    # Telemetry from Robot
    print(f"Alignment: {isAligned}, Center X: {pos[0]}, Distance: {pos[1]}")
    
    # Main Movement Logic
    if canOrbit(errorX, errorY, isAligned):
        errorX, errorY = robot_class.move_to_coarse(pos, True)
    else:
        errorX, errorY = robot_class.move_to_coarse(pos, False)
    
    return errorX, errorY
    
# Test Functions

def test1_color(robot_class):
    '''
    This is just project 0 in this project. Except it chases a brick
    '''

    detector, errorX, errorY, isAligned = initTest(robot_class)

    while True:
        try:
            img = robot_class.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        brick, center = updateImage(detector, img)

        if center:
            errorX, errorY = move_approach(robot_class, detector, brick, center, errorX, errorY)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

def test2_color(robot_class):
    '''
    Rotates in place until a block is found (with the matching color mask)
    Uses a P control loop to approach the block, uses a pre-programmed 
    fine movement state machine to approach the block until it's grabbed.
    '''
    # Detector Init
    detector = Detect()

    timeStart = time.time()
    pos_y = 0.4
    errorX = 10
    errorY = 10
    distList = []
    xList = []
    isAligned = False
    isOrbiting = False

    # Tolerances
    xTol = 10 # Pixels
    yTol = 0.01 # Meters
    rotTol = 30

    robot_class.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            img = robot_class.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        mask = detector.detect_object(img, detector.RED)
        edges = detector.edges(mask)
        center = detector.center(edges)
        
        # If you want the area detecting code
        brick = detector.isolate_brick(edges)

        # This is line based measuring
        # edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                
        if not(robot_class.isGrip):
            if center:
                # Output tower center pixel
                center_x, center_y = center
                
                # Get position on X and average it
                pos_x = center_x-detector.FRAME_CENTER_X
                xList.insert(0, pos_x)
                if len(xList) > 5:
                    xList.pop()

                # This is the final average that it takes
                pos_x = sum(xList)/5

                # Distance using Lines
                # vertical_lines = detector.sides(edges, center_x)
                # edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                # for x1, y1, x2, y2 in vertical_lines:
                #     cv2.line(edges_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green
                # cv2.circle(edges_bgr, center, 10, (0, 0, 255), -1)  # Red dot
                # pos_y = detector.distance_lines(detector.line_length(vertical_lines))

                # Area Detection Code
                # if pos_y > 0:
                
                # Averages the Distance Values
                pos_y = detector.distance_area_far(brick)
                distList.insert(0,pos_y)
                if len(distList) > 5:
                    distList.pop()

                # This is the final average that it takes
                pos_y = sum(distList)/5

                isAligned = detector.orientation(brick)

                # else:
                #     pos_y = detector.distance_area_near(brick)

                print(f"Alignment: {isAligned}, Orbit: {isOrbiting}, Center X: {pos_x}, Distance: {pos_y}")
                # print(detector.line_length(vertical_lines))

                if abs(pos_x) < rotTol:
                    pos = [pos_x, 0, pos_y]
                    rot = [0, 0, 0]
                    
                    errorY, errorX = robot_class.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if isAligned:
                            robot_class.isGrip = True
                            gripThread = threading.Thread(target=robot_class.move_to_fine)
                            gripThread.start()
                        else:
                            errorY, errorX = robot_class.move_to_coarse(pos, rot, True)
                            isOrbiting = True

                else:
                    robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
            else:
                # if not(robot_class.isLost):
                #     timeStart = time.time()
                #     robot_class.isLost = True
                # if time.time()-timeStart < 18:
                robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
                # else:
                #     raise Exception("Timeout: no blocks found")

        # Display the captured frame
        cv2.imshow('Camera', brick) # brick for area to distance

        if cv2.waitKey(1) == ord('q'):
            break

def test3_color(tobot_class):
    '''
    Picks up red block and goes to pad in a pre-programmed sequence

    Rotates in place until its target is found and then it places it away from 
    its inital spot.

    Switches from a coarse motion, P-loop based, to a fine motion, pre-programmed
    state machine.
    '''
    # Detector Init
    detector = Detect()
    robot_class.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    timeStart = time.time()
    errorX = 10
    errorY = 10
    xTol = 10 # Pixels
    yTol = 0.01 # Meters
    rotTol = 30

    distList = []
    xList = []
    isAligned = False
    isOrbiting = False

    findRed = True
    pickRed = False
    findPad = False

    while True:
        try:
            img = robot_class.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(img, detector.RED)
        if findPad:
            mask = detector.detect_object(img, detector.PAPER_ORANGE)
        edges = detector.edges(mask)
        center = detector.center(edges)

        brick = detector.isolate_brick(edges)

        if not(robot_class.isGrip):
            if center:
                # Output tower center pixel
                center_x, center_y = center
                
                pos_x = center_x-detector.FRAME_CENTER_X
                # Get position on X and average it
                xList.insert(0, pos_x)
                if len(xList) > 5:
                    xList.pop()

                # This is the final average that it takes
                pos_x = sum(xList)/5

                pos_y = detector.distance_area_far(brick)
                distList.insert(0,pos_y)
                if len(distList) > 5:
                    distList.pop()

                # This is the final average that it takes
                pos_y = sum(distList)/5

                isAligned = detector.orientation(brick)

                print(f"Alignment: {isAligned}, Orbit: {isOrbiting}, Pad: {findPad}, Red: {findRed}, Distance: {pos_y}")

                if abs(pos_x) < rotTol and not(pos_y != pos_y):
                    pos = [pos_x, 0, pos_y]
                    rot = [0, 0, 0]
                    errorY, errorX = robot_class.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if isAligned:
                            robot_class.isGrip = True

                            if findRed:
                                findRed = False
                                pickRed = True
                                findPad = True

                            gripThread = threading.Thread(target=robot_class.move_to_fine2)
                            gripThread.start()
                        else:
                            errorY, errorX = robot_class.move_to_coarse(pos, rot, True)
                            isOrbiting = True
                else:
                    robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

            else:
                robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

def test3b_color(robot_class):
    '''
    Picks up blocks in a pre-programmed sequence, (red to green or green to red).

    Rotates in place until its target is found and then it places it away from 
    its inital spot.

    Switches from a coarse motion, P-loop based, to a fine motion, pre-programmed
    state machine.
    '''
    # Detector Init
    detector = Detect()
    robot_class.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    timeStart = time.time()
    errorX = 10
    errorY = 10
    yTolfine = 0.01 # Meters
    xTol = 3 # Pixels
    yTol = 0.05 # Meters
    rotTol = 30

    distList = []
    xList = []
    isAligned = False
    isOrbiting = False

    findRed = True
    pickRed = False
    findGreen = False
    pickGreen = False

    while True:
        try:
            img = robot_class.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        if findRed:
            mask = detector.detect_object(img, detector.BRICK_RED)
        if findGreen:
            mask = detector.detect_object(img, detector.BRICK_GREEN)
        edges = detector.edges(mask)
        center = detector.center(edges)
        brick = detector.isolate_brick(edges)

        if not(robot_class.isGrip):
            if center:
                # Output tower center pixel
                center_x, center_y = center
                
                pos_x = center_x-detector.FRAME_CENTER_X
                # Get position on X and average it
                xList.insert(0, pos_x)
                if len(xList) > 5:
                    xList.pop()

                # This is the final average that it takes
                pos_x = sum(xList)/5

                pos_y = detector.distance_area_far(brick)
                distList.insert(0,pos_y)
                if len(distList) > 5:
                    distList.pop()

                # This is the final average that it takes
                pos_y = sum(distList)/5

                isAligned = detector.orientation_avg(brick)

                print(f"Alignment: {isAligned}, Orbit: {isOrbiting}, Green: {findGreen}, Red: {findRed}, Distance: {pos_y}")

                if abs(pos_x) < rotTol and not(pos_y != pos_y):
                    pos = [pos_x, 0, pos_y]
                    rot = [0, 0, 0]
                    errorY, errorX = robot_class.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if abs(errorY) <= yTolfine and isAligned:
                            robot_class.isGrip = True

                            if findRed:
                                findRed = False
                                pickRed = True
                                findGreen = True

                            if pickGreen and findGreen:
                                findGreen = False
                                pickGreen = True
                                findRed = True

                            gripThread = threading.Thread(target=robot_class.move_to_fine)
                            gripThread.start()
                        else:
                            errorY, errorX = robot_class.move_to_coarse(pos, rot, True)
                            isOrbiting = True
                else:
                    robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

            else:
                robot_class.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    # Robot Init
    r1 = motion()

    try:
        test1_color(r1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        shutdown(r1)