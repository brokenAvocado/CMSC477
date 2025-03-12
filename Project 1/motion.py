import pupil_apriltags
import cv2
import numpy as np
import time
import traceback
from queue import Empty
import robomaster
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from djikstra import DJI
import multiprocessing 

class AprilTagDetector: # Given
    def __init__(self, K, family="tag36h11", threads=2, marker_size_m=0.16):
        self.camera_params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        self.marker_size_m = marker_size_m
        self.detector = pupil_apriltags.Detector(family, threads)

    def find_tags(self, frame_gray):
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections

# X, Y coordinates, X and Y orientation
tagDict = {
    30: [[0.532, 1.995, 0], [1, 1, 0]],
    31: [[0.798, 1.995, 0], [1, 1, np.pi]],
    32: [[0.532, 1.463, 0], [1, 1, 0]],
    33: [[0.798, 1.463, 0], [1, 1, np.pi]],
    34: [[0.665, 1.064, 0], [1, 1, np.pi*3/2]],
    35: [[1.197, 2.128, 0], [1, 1, np.pi*3/2]],
    36: [[1.729, 2.128, 0], [1, 1, np.pi*3/2]],
    37: [[1.463, 1.33, 0], [1, 1, np.pi/2]],
    38: [[1.33, 0.931, 0], [1, 1, 0]],
    39: [[1.596, 0.931, 0], [1, 1, np.pi]],
    40: [[1.33, 0.399, 0], [1, 1, 0]],
    41: [[1.596, 0.399, 0], [1, 1, np.pi]],
    42: [[2.128, 1.995, 0], [1, 1, 0]],
    43: [[2.394, 1.995, 0], [1, 1, np.pi]],
    44: [[2.128, 1.463, 0], [1, 1, 0]],
    45: [[2.394, 1.463, 0], [1, 1, np.pi]],
    46: [[2.261, 1.064, 0], [1, 1, np.pi*3/2]]
}

scalingFactor = 0.266/3

'''
Desc: gets the april tag orientation in Euler Angles (Roll, Pitch, Yaw)
and position of the tag with corrected pitch (accounting for camera)

Input: detection object (comes from AprilTagDetector)
Output: t_ca (position array 1x3), and rotation (1x3)
'''
def get_pose_apriltag_in_camera_frame(detection):
    R_ca = detection.pose_R
    t_ca = detection.pose_t

    roll = np.arctan2(R_ca[1][0],R_ca[0][0])
    yaw = np.arctan2(-R_ca[2][0],np.sqrt(R_ca[2][1]**2+R_ca[2][2]**2))
    pitch = np.arctan2(R_ca[2][1],R_ca[2][2])
    const = 1 #180/np.pi
    
    rotation = [const*roll, const*yaw, const*pitch]

    t_ca = t_ca.flatten()
    t_ca[2] = t_ca[2]*np.cos(pitch)
    t_ca[0] = t_ca[0]*np.cos(yaw)

    return t_ca, rotation

def draw_detections(frame, detections): # Given
    for detection in detections:
        pts = detection.corners.reshape((-1, 1, 2)).astype(np.int32)

        frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

        top_left = tuple(pts[0][0])  # First corner
        top_right = tuple(pts[1][0])  # Second corner
        bottom_right = tuple(pts[2][0])  # Third corner
        bottom_left = tuple(pts[3][0])  # Fourth corner
        cv2.line(frame, top_left, bottom_right, color=(0, 0, 255), thickness=2)
        cv2.line(frame, top_right, bottom_left, color=(0, 0, 255), thickness=2)

'''
Desc: Proportional-Integral control loop for the robot to follow an April tag

Input:
- robot_pos: Robot current position
- dest_pos: Desired destination
- sumX, sumY, sumZ: summation term for the integral part of the control loop
- dt: delta time for the integral loop
Output: 
'''
def control_loop(ep_robot, ep_chassis, robot_pos, dest_pos, Rpose, angle):
    Px = .8
    offsetX = 0

    Py = Px
    offsetY = 0
    
    Pz = 300

    # pi/2 = straight up +y
    # 0 = looking to the +x (starting position)
    # pi = looking to the -x

    errorX = dest_pos[0]-robot_pos[0]-offsetX
    errorY = dest_pos[1]-robot_pos[1]-offsetY

    if(abs(errorX) < 0.02):
        errorX = 0
    if(abs(errorY) < 0.02):
        errorY = 0

    velx = Px*(errorX)
    vely = -Py*(errorY)
    velz = Pz*(Rpose[1])
    
    abs_velx = velx*np.cos(angle) - vely*np.sin(angle)
    abs_vely = velx*np.sin(angle) + vely*np.cos(angle)

    # if(abs(errorX) < 0.02):
    #     velx = 0
    # if(abs(errorY) < 0.02):
    #     vely = 0
    # if(abs(Rpose[1]) < 0.044):
    #     velz = 0

    ep_chassis.drive_speed(x=abs_velx, y=abs_vely, z=velz, timeout = 0.05)

    return errorX, errorY

'''
Desc: finds the nearest april tag in the camera view based on pure distance to robot

Input: detections
Output: closestTag object
'''
def closest(detections):
    closestDist = float('inf')
    closestTag = 0
    yaw_lim = np.pi/3

    for tag in detections:
        pos, rot = get_pose_apriltag_in_camera_frame(tag)
        if np.linalg.norm([pos[0], pos[2]]) < closestDist:
            if abs(rot[1]) < yaw_lim:
                closestDist = np.linalg.norm([pos[0], pos[2]])
                closestTag = tag
    if closestTag == 0:
        return None
    
    return closestTag

'''
Desc: converts the position in the maze to the real world

Input: 
- tag: the object with the information on the tag
Output:
- abs_x, abs_y: the position of the robot in real world
'''
def relative2world(tag):
    tag_id = tag.tag_id
    pos, rot = get_pose_apriltag_in_camera_frame(tag)
    r_x = pos[2]
    r_y = pos[0]

    # yaw = rot[1]
    yaw = 0
    
    tgx = tagDict[tag_id][0][0]
    tgy = tagDict[tag_id][0][1]
    tgox = tagDict[tag_id][1][0]
    tgoy = tagDict[tag_id][1][1]
    multiple = tagDict[tag_id][1][2]//(np.pi/2)
    angle = tagDict[tag_id][1][2]

    x_rel = r_x*np.cos(yaw) - r_y*np.sin(yaw)
    y_rel = r_x*np.sin(yaw) + r_y*np.cos(yaw)
    # dims_unrotated = np.array([[x_rel*tgox, 0],[0, y_rel*tgoy]])
    # dims_rotated = np.rot90(dims_unrotated, multiple)
    x_rel_rot = x_rel*np.cos(angle) - y_rel*np.sin(angle)
    y_rel_rot = x_rel*np.sin(angle) + y_rel*np.cos(angle) 
    
    x_abs = tgx-x_rel_rot*tgox
    y_abs = tgy+y_rel_rot*tgoy
    return x_abs, y_abs

def test_tag(ep_camera, apriltag):
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)

        if len(detections) > 0:
            tag = closest(detections)
            x, y = relative2world(tag)
            _, rot = get_pose_apriltag_in_camera_frame(tag)
            print(rot)

        draw_detections(img, detections)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

'''
Without using motion commands, track the robot going through a maze.

Input: april tag
Output: mapping of the robot in the maze

'''

def detect_tag_loop(ep_camera, apriltag):
    # Constants
    yaw_lim = np.pi/3

    # Init Graphing Functions
    pathfinding = DJI("Project 1\\Lab1.csv")
    pathfinding.initPlot()
    fig, ax = plt.subplots()

    graph = plt.plot(np.array(pathfinding.xs)*scalingFactor, np.array(pathfinding.ys)*scalingFactor,'ko')
    graph = ax.plot([],[],'go')[0]
    ax.set(xlim=[0, 3.5],ylim=[0, 3.5])
    plt.draw()
    plt.pause(0.0001)

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)

        if len(detections) > 0:
            tag = closest(detections)
            x, y = relative2world(tag)
            print(f'Tag ID: {tag.tag_id}| Robot X: {x}| Robot Y: {y}')

            # Graphing Functions
            graph.set_xdata(x)
            graph.set_ydata(y)
            plt.draw()
            plt.pause(0.00001)

        draw_detections(img, detections)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

def motionControl(ep_robot, ep_chassis, ep_camera, apriltag):
    errorSumX = 0
    errorSumY = 0
    errorSumZ = 0

    plotStart = time.time()
    runTime = 0

    timeArray = []
    errorXPlot = []
    errorYPlot = []

    while True:
        startTime = time.time()

        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)

        detections = apriltag.find_tags(gray)

        if len(detections) > 0:
            # assert len(detections) == 1 # Assume there is only one AprilTag to track
            detection = detections[0]

            t_ca, R_ca = get_pose_apriltag_in_camera_frame(detection)
            # print('t_ca', t_ca)
            # print('R_ca', R_ca)
            # print("poll")
            oldRunTime = runTime
            runTime = time.time()
            # sumX, sumY, sumZ, errorX, errorY = control_loop(ep_robot, ep_chassis, t_ca, R_ca, errorSumX, errorSumY, errorSumZ, 0.03)
            # errorSumX = sumX
            # errorSumY = sumY
            # errorSumZ = sumZ
        
        draw_detections(img, detections)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            plt.scatter(timeArray, errorXPlot)
            # plt.scatter(timeArray, errorYPlot)
            plt.xlabel("Time (sec)")
            plt.ylabel("Error (sec)")
            plt.show()
            break

def maze_movement(ep_robot, ep_chassis, ep_camera, apriltag):
    # Constants
    plotStart = time.time()
    runTime = 0

    timeArray = []
    errorXPlot = []
    errorYPlot = []
    yaw_lim = np.pi/3

    # Init Graphing Functions
    pathfinding = DJI("Project 1\\Lab1.csv")
    pathfinding.initPlot()
    pathfinding.search()

    path_xcoord = np.array(pathfinding.pathx)*scalingFactor
    path_ycoord = np.array(pathfinding.pathy)*scalingFactor
    path_ind = 0

    fig, ax = plt.subplots()
    graph = plt.plot(np.array(pathfinding.xs)*scalingFactor, np.array(pathfinding.ys)*scalingFactor,'ko')
    graph = plt.plot(path_xcoord, path_ycoord, 'ro', markersize=3)
    graph = ax.plot([],[],'go')[0]
    ax.set(xlim=[0, 3.5],ylim=[0, 3.5])


    plt.draw()
    plt.pause(0.0001)

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)
        detections = apriltag.find_tags(gray)

        tag = closest(detections)
        if tag != 0:
            x, y = relative2world(tag)
            _, Rpose = get_pose_apriltag_in_camera_frame(tag)
            print(f'Tag ID: {tag.tag_id}| Robot X: {x}| Robot Y: {y}')

            errorX, errorY = control_loop(ep_robot, ep_chassis, [x, y], [path_xcoord[path_ind], path_ycoord[path_ind]], Rpose)
            print(f'Errors| Robot X: {errorX}, Robot Y: {errorY}')

            # Graphing Functions
            graph.set_xdata(x)
            graph.set_ydata(y)
            plt.draw()
            plt.pause(0.00001)

        draw_detections(img, detections)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == '__main__':
    robomaster.config.ROBOT_IP_STR = "192.168.50.121"
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")

    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    K = np.array([[314, 0, 320], [0, 314, 180], [0, 0, 1]]) # Camera focal length and center pixel
    marker_size_m = 0.153 # Size of the AprilTag in meters
    apriltag = AprilTagDetector(K, threads=2, marker_size_m=marker_size_m)

    try:
        # test_tag(ep_camera, apriltag)
        # detect_tag_loop(ep_camera, apriltag)
        maze_movement(ep_robot, ep_chassis, ep_camera, apriltag)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        print('Waiting for robomaster shutdown')
        ep_camera.stop_video_stream()
        ep_robot.close()

