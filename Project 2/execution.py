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

def test1():
    '''
    This is just project 0 in this project.
    '''

    # AprilTag Init
    apriltag = AprilTagDetector()

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    errorX = 10
    errorY = 10

    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)

        tags = apriltag.find_tags(gray)
        if len(tags) > 0:
            tag = tags[0]
            pos, rot = apriltag.get_pose_apriltag_in_camera_frame(tag)
            errorX, errorY = r1.move_to_coarse(pos, rot)

        apriltag.draw_detections(img, tags)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

def test2():
    '''
    This is project 0 motion with an added search tags feature (rotates until tag found).
    '''
    # AprilTag Init
    apriltag = AprilTagDetector()

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    errorX = 10
    errorY = 10
    xTol = 0.02
    yTol = xTol

    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)

        tags = apriltag.find_tags(gray)
        if not(r1.isGrip):
            if len(tags) > 0:
                r1.isLost = False
                tag = tags[0]
                pos, rot = apriltag.get_pose_apriltag_in_camera_frame(tag)
                errorX, errorY = r1.move_to_coarse(pos, rot)
                if abs(errorX) <= xTol and abs(errorY) <= yTol:
                    r1.isGrip = True
                    gripThread = threading.Thread(target=r1.move_to_fine)
                    gripThread.start()
            else:
                if not(r1.isLost):
                    timeStart = time.time()
                    r1.isLost = False
                if time.time()-timeStart < 18:
                    r1.ep_chassis.drive_speed(x=0, y=0, z=40, timeout = 0.05)
                else:
                    raise Exception("Timeout: no tags found")

        apriltag.draw_detections(img, tags)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

def test3():
    # AprilTag Init
    apriltag = AprilTagDetector()

    init = False
    objects = []
    errorX = 10
    errorY = 10

    startTime = time.time()
    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)


    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)

        tags = apriltag.find_tags(gray)

        if (time.time()-startTime <= 12):
            r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
            for tag in tags:
                if not(tag.tag_id in objects): 
                    objects.append(tag)
        else:
            target = objects[0]
            for tag in tags:
                if target == tag.tag_id:
                    pos, rot = apriltag.get_pose_apriltag_in_camera_frame(target)
                    if errorX > 0.03 and errorY > 0.03:
                        errorX, errorY = r1.move_to_coarse(pos, rot)
                    else:
                        r1.move_to_fine()
                        r1.lgr()
                        time.sleep(1)
                        r1.move_away()
                else:
                    r1.ep_chassis.drive_speed(x=0, y=0, z=40, timeout = 0.05)

        apriltag.draw_detections(img, tags)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

def test1_color():
    '''
    This is just project 0 in this project.
    '''
    # Detector Init
    detector = Detect()

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    errorX = 10
    errorY = 10
    isAligned = True

    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(img, detector.RED)
        edges = detector.edges(mask)
        center = detector.center(edges)
        brick = detector.isolate_brick(edges)

        if center:
            # Output tower center pixel
            center_x, center_y = center
            isAligned = detector.orientation(brick)

            pos_x = center_x-detector.FRAME_CENTER_X
            pos_y = detector.distance_area_far(brick)

            print(f"Alignment: {isAligned}, Center X: {pos_x}, Distance: {pos_y}")
            # print(detector.line_length(vertical_lines))
            
            pos = [pos_x, 0, pos_y]
            rot = [0, 0, 0]

            if abs(errorX) <= 5 and abs(errorY) <= 0.1 and not(isAligned):
                errorX, errorY = r1.move_to_coarse(pos, rot, True)
            else:
                errorX, errorY = r1.move_to_coarse(pos, rot, False)

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

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
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
                
        if not(r1.isGrip):
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
                    
                    errorY, errorX = r1.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if isAligned:
                            r1.isGrip = True
                            gripThread = threading.Thread(target=r1.move_to_fine)
                            gripThread.start()
                        else:
                            errorY, errorX = r1.move_to_coarse(pos, rot, True)
                            isOrbiting = True

                else:
                    r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
            else:
                # if not(r1.isLost):
                #     timeStart = time.time()
                #     r1.isLost = True
                # if time.time()-timeStart < 18:
                r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
                # else:
                #     raise Exception("Timeout: no blocks found")

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
    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(img, detector.RED)
        if findPad:
            mask = detector.detect_object(img, detector.PAPER_ORANGE)
        edges = detector.edges(mask)
        center = detector.center(edges)

        brick = detector.isolate_brick(edges)

        if not(r1.isGrip):
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
                    errorY, errorX = r1.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if isAligned:
                            r1.isGrip = True

                            if findRed:
                                findRed = False
                                pickRed = True
                                findPad = True

                            gripThread = threading.Thread(target=r1.move_to_fine2)
                            gripThread.start()
                        else:
                            errorY, errorX = r1.move_to_coarse(pos, rot, True)
                            isOrbiting = True
                else:
                    r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

            else:
                r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

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
    # Detector Init
    detector = Detect()
    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue
        
        if findRed:
            mask = detector.detect_object(img, detector.RED)
        if findGreen:
            mask = detector.detect_object(img, detector.GREEN)
        edges = detector.edges(mask)
        center = detector.center(edges)

        brick = detector.isolate_brick(edges)

        if not(r1.isGrip):
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

                isAligned = detector.orientation(brick, 15)

                print(f"Alignment: {isAligned}, Orbit: {isOrbiting}, Green: {findGreen}, Red: {findRed}, Distance: {pos_y}")

                if abs(pos_x) < rotTol and not(pos_y != pos_y):
                    pos = [pos_x, 0, pos_y]
                    rot = [0, 0, 0]
                    errorY, errorX = r1.move_to_coarse(pos, rot, False)
                    isOrbiting = False

                    if abs(errorX) <= xTol and abs(errorY) <= yTol:
                        if abs(errorY) <= yTolfine and isAligned:
                            r1.isGrip = True

                            if findRed:
                                findRed = False
                                pickRed = True
                                findGreen = True

                            if pickGreen and findGreen:
                                findGreen = False
                                pickGreen = True
                                findRed = True

                            gripThread = threading.Thread(target=r1.move_to_fine)
                            gripThread.start()
                        else:
                            errorY, errorX = r1.move_to_coarse(pos, rot, True)
                            isOrbiting = True
                else:
                    r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

            else:
                r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        if cv2.waitKey(1) == ord('q'):
            break

def detectTest():
    detector = Detect()
    detector.set_lower_mask(114, .6, .2)
    detector.set_upper_mask(154, 1, 1)

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            frame = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        masked = detector.mask_image_pls(frame)
        edges = detector.edges(masked)
        center = detector.center(edges, 15)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if center:
            center_x, center_y = center
            vertical_lines = detector.sides(edges, center_x)

            # Draw on color image
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            for x1, y1, x2, y2 in vertical_lines:
                cv2.line(edges_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green
            cv2.circle(edges_bgr, center, 10, (0, 0, 255), -1)  # Red dot

            # cv2.imshow('Camera', edges_bgr)
            print(f"Distance: {detector.distance(detector.line_length(vertical_lines))}")
            # print(detector.line_length(vertical_lines))

        # Display the captured frame
        cv2.imshow('Camera', edges_bgr)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

def areaTest():
    detector = Detect()
    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            frame = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(frame, detector.PAPER_ORANGE)
        edges = detector.edges(mask)

        brick = detector.isolate_brick(edges)
        # distance = detector.distance_area(brick)

        #print(f"Distance: {distance}")

        # Display the captured frame
        cv2.imshow('Camera', brick)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    # Robot Init
    r1 = motion()
    r1.gripper_open()
    r1.gripper_open()

    try:
        test3b_color()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        r1.ep_arm.unsub_position()
        print('Waiting for robomaster shutdown')
        r1.ep_camera.stop_video_stream()
        r1.ep_robot.close()