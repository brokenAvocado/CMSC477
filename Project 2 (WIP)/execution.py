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
import os

from motion import motion
from motion import AprilTagDetector
from detect import Detect
import threading

# Perception Functions

def updateImage(img, objType):
    '''
    Used to update the images from the detector with masks and edges
    Returns the masked brick object
    '''
    mask = detector.detect_object(img, objType)
    crop_mask = detector.crop_image(mask)
    edges = detector.edges(crop_mask)
    brick = detector.isolate_brick(edges)
    center = detector.center(brick)
    brick_disp = detector.draw_center(brick)

    return brick, center, brick_disp

def updateDistances(img, center_obj):
    '''
    Gets the distance of block from centroid 
    Returns the positions (x = pixels, y = meters)
    '''
    # Get the pixel that the brick centroid is
    center_x, center_y = center_obj
    pos_x = center_x-detector.FRAME_CENTER_X
    # Get distance to brick
    pos_y = detector.distance_area_far(img)

    return (pos_x, pos_y)

def blockFound(img, center_obj, rotTol=30):
    if center_obj:
        pos = updateDistances(img, center_obj)
        if pos[0] < rotTol:
            return True
    return False

# System Functions:

def shutdown():
    '''
    Sequence for stopping the robot 
    '''
    robo.ep_arm.unsub_position()
    print('Waiting for robomaster shutdown')
    robo.ep_camera.stop_video_stream()
    robo.ep_robot.close()

def initVars():
    errorX = 10
    errorY = 10

    return errorX, errorY

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

# Movement Functions

def canOrbit(x, y, alignment, x_tol=12, y_tol=0.05):
    '''
    Checks for the conditions to orbit around the object
    '''
    if abs(x) < x_tol and abs(y) < y_tol and not(alignment):
        return True
    return False
    
def canGrab(x, y, brickMask, x_tol=10, y_tol=0.008, state = None):
    alignment = detector.orientation_avg(brickMask, 15, 15)

    if state == 2 or state == 4:
        x_tol = 20.0
        y_tol = 0.008
        # alignment = True

    if abs(x) < x_tol and abs(y) < y_tol and (alignment):
        return True
    return False
    
def move_approach(brickMask, center_obj, errorX, errorY, state = None):
    pos = updateDistances(brickMask, center_obj)
    isAligned = detector.orientation_avg(brickMask, 15, 15)
    isOrbiting = canOrbit(errorX, errorY, isAligned)

    if state == 2 or state == 4:
        offset_dist = 0.55
        ignoreOrbit = False
    else:
        offset_dist = 0.3
        ignoreOrbit = False

    # Main Movement Logic
    if isOrbiting and not(ignoreOrbit):
        errorY, errorX = robo.move_orbit(pos, Px = 0.4, Py = 0.003)
    else:
        errorY, errorX = robo.move_to_coarse(pos, False, offsetX = offset_dist, Px = 0.4, Py = 0.003)

    return errorX, errorY, pos[0], pos[1], isAligned

# def check_grab(brickMask, alignCount, alignMax = 10):
#     go_grab = False
#     isAligned = detector.orientation_avg(brickMask, 15, 15)
    
#     if isAligned:
#         alignCount += 1

#     if alignCount > alignMax:
#         go_grab = True

#     return go_grab, alignCount
    
def move_grab(state):
    robo.isGrip = True
    ind = state

    if ind == 0:
        gripThread = threading.Thread(target=robo.move_block)
        gripThread.start()
    elif ind == 1:
        gripThread = threading.Thread(target=robo.move_to_block)
        gripThread.start()
    elif ind == 2:
        gripThread = threading.Thread(target=robo.move_to_pad)
        gripThread.start()
    elif ind == 3:
        gripThread = threading.Thread(target=robo.move_to_block)
        gripThread.start()
    elif ind == 4:
        gripThread = threading.Thread(target=robo.move_to_pad)
        gripThread.start()

def search_block():
    robo.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

def state_change(state):
    '''
    The state is constructed of the following tasks:
    0 - Find Red
    1 - Find Green
    2 - Find Pad
    3 - Find Red 2
    4 - Find Pad 2
    '''

    ind = state.index(1)
    if ind < len(state):
        state[ind+1] = state[ind]
        state[ind] = 0

    return state

def color_switch(state):
    ind = state

    if ind == 0:
        return detector.BRICK_RED
    elif ind == 1:
        return detector.BRICK_GREEN
    elif ind == 2:
        return detector.PAPER_ORANGE
    elif ind == 3:
        return detector.BRICK_RED
    elif ind == 4:
        return detector.PAPER_PURPLE
    return detector.PAPER_PURPLE

# Test Functions

def test1_color():
    '''
    This is just project 0 in this project. Except it chases a brick
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    # Initialize our variables
    errorX = 10 # Just so we don't trigger the conditional early, set arbitrarily to 10
    errorY = 10
    count = 0
    tol = 5
    dir = False
    isAligned = True

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        brick, center = updateImage(img, detector.BRICK_GREEN)

        if center:
            errorX, errorY = move_approach(brick, center, errorX, errorY)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

def test2_color():
    '''
    Rotates in place until a block is found (with the matching color mask)
    Uses a P control loop to approach the block, uses a pre-programmed 
    fine movement state machine to approach the block until it's grabbed.
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    robo.gripper_open()
    robo.gripper_open()
    errorX, errorY = initVars()
    state = [1]

    while True:
        try:
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        brick, center = updateImage(img, detector.BRICK_GREEN)
        
        # Top most conditional checks if the robot is still in thread (move_to_fine)
        if not(robo.isGrip):
            if blockFound(brick, center):
                if canGrab(errorX, errorY, brick):
                    move_grab(state)
                else:
                    errorX, errorY = move_approach(brick, center, errorX, errorY)
            else:
                search_block()
                errorX, errorY = initVars()

        # Display the captured frame
        cv2.imshow('Camera', brick) # brick for area to distance

        if cv2.waitKey(1) == ord('q'):
            break

def test3_color():
    '''
    Picks up red block and goes to pad in a pre-programmed sequence

    Rotates in place until its target is found and then it places it away from 
    its inital spot.

    Switches from a coarse motion, P-loop based, to a fine motion, pre-programmed
    state machine.
    '''
    # Detector Init
    detector = Detect()
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
            img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(img, detector.BRICK_RED)
        if findPad:
            mask = detector.detect_object(img, detector.PAPER_ORANGE)
        edges = detector.edges(mask)
        center = detector.center(edges)

        brick = detector.isolate_brick(edges)

        if not(robo.isGrip):
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
                    errorY, errorX = robo.move_to_coarse(pos, rot, False)
                    print("here")
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if isAligned:
                            robo.isGrip = True

                            if findRed:
                                findRed = False
                                pickRed = True
                                findPad = True

                            gripThread = threading.Thread(target=robo.move_to_fine2)
                            gripThread.start()
                        else:
                            errorY, errorX = robo.move_to_coarse(pos, rot, True)
                            isOrbiting = True
                else:
                    robo.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

            else:
                robo.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

def test3b_color():
    '''
    Picks up blocks in a pre-programmed sequence, (red to green or green to red).

    Rotates in place until its target is found and then it places it away from 
    its inital spot.

    Switches from a coarse motion, P-loop based, to a fine motion, pre-programmed
    state machine.
    '''
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    robo.gripper_open()
    robo.gripper_open()
    # robo.lgr()
    errorX, errorY = initVars()
    state = 0
    x = 0
    y = 0
    alignment = False

    path = open_new_file("Project 2")
    with open(path, 'w') as data:
        while True:
            timeStart = time.time()
            try:
                img = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            except Empty:
                time.sleep(0.001)
                continue

            color = color_switch(state)
            brick, center, display = updateImage(img, color)

            # print(color)

            # Top most conditional checks if the robot is still in thread (move_to_fine)
            if not(robo.isGrip):
                if blockFound(brick, center):
                    if canGrab(errorX, errorY, brick, state=state):
                        move_grab(state)
                        state += 1
                        errorX, errorY = initVars()
                    else:
                        errorX, errorY, x, y, alignment = move_approach(brick, center, errorX, errorY, state)
                else:
                    search_block()
                    errorX, errorY = initVars()

            # Telemetry from Robot
            print(f"isAligned: {alignment}, State: {state}, Center X: {x}, Distance: {y}")
            save_data(data, f"{time.time() - timeStart}\n")
            # print(time.time() - timeStart)

            # Display the captured frame
            cv2.imshow('Camera', brick)

            if cv2.waitKey(1) == ord('q'):
                break

def detectTest():
    color = detector.PAPER_ORANGE
    robo.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            frame = robo.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        # brick, center, display = updateImage(frame, color)
        # pos = updateDistances(brick, center)

        # print(f"Distance, {pos[1]}")
            
        # Display the captured frame
        cv2.imshow('Camera', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    # Robot Init
    robo = motion()
    detector = Detect()

    try:
        test3b_color()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        shutdown()