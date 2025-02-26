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
from djikstra import DJI

class AprilTagDetector:
    def __init__(self, K, family="tag36h11", threads=2, marker_size_m=0.16):
        self.camera_params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        self.marker_size_m = marker_size_m
        self.detector = pupil_apriltags.Detector(family, threads)

    def find_tags(self, frame_gray):
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections

def get_pose_apriltag_in_camera_frame(detection):
    R_ca = detection.pose_R
    t_ca = detection.pose_t
    return t_ca.flatten(), R_ca

def draw_detections(frame, detections):
    for detection in detections:
        pts = detection.corners.reshape((-1, 1, 2)).astype(np.int32)

        frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

        top_left = tuple(pts[0][0])  # First corner
        top_right = tuple(pts[1][0])  # Second corner
        bottom_right = tuple(pts[2][0])  # Third corner
        bottom_left = tuple(pts[3][0])  # Fourth corner
        cv2.line(frame, top_left, bottom_right, color=(0, 0, 255), thickness=2)
        cv2.line(frame, top_right, bottom_left, color=(0, 0, 255), thickness=2)

def control_loop(ep_robot_loop, ep_chassis_loop, Tpose, Rpose, sumX, sumY, sumZ, dt):
    Px = 3
    offsetX = .5
    Py = 3
    offsetY = 0
    Pz = 300

    Ix = 3
    Iy = 3
    # Iz = 0.1

    errorX = Tpose[2]-offsetX
    errorY = Tpose[0]-offsetY

    if(abs(errorX) < 0.02):
        errorX = 0
    if(abs(errorY) < 0.02):
        errorY = 0

    sumX += errorX*dt
    sumY += errorY*dt
    sumZ += Rpose[1]*dt

    # if (abs(sumX) > 0.2):
    #     sumX = 0.2
    # if (abs(sumY) > 0.2):
    #     sumY = 0.2

    print(str(sumX) + ", " + str(sumY) + "," + str(sumZ))

    velx = Px*(errorX) + Ix*sumX
    vely = Py*(errorY) + Iy*sumY
    velz = Pz*Rpose[1]

    if(abs(errorX) < 0.02):
        velx = 0
    if(abs(errorY) < 0.02):
        vely = 0
    if(abs(Rpose[1]) < 0.044):
        velz = 0

    ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.02)

    return sumX, sumY, sumZ, errorX, errorY

def rotationToEuler(Rpose):
    roll=np.arctan2(Rpose[1][0],Rpose[0][0])
    yaw=np.arctan2(-Rpose[2][0],np.sqrt(Rpose[2][1]**2+Rpose[2][2]**2))
    pitch=np.arctan2(Rpose[2][1],Rpose[2][2])
    const = 1 #180/np.pi

    return [const*roll, const*yaw, const*pitch]

def detect_tag_loop(ep_camera, apriltag):
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
            # assert len(detections) == 1 # Assume there is only one AprilTag to track
            for tags in detections: 
                print(tags.tag_id)

                t_ca, R_ca = get_pose_apriltag_in_camera_frame(tags)
                R_ca = rotationToEuler(R_ca)
                print("Position = " + str(t_ca))
                print("Rotation = " + str(R_ca))

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

            # print(rotationToEuler(R_ca))
            R_ca = rotationToEuler(R_ca)
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
            

if __name__ == '__main__':
    # More legible printing from numpy.
    np.set_printoptions(precision=3, suppress=True, linewidth=120)

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
        detect_tag_loop(ep_camera, apriltag)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        print('Waiting for robomaster shutdown')
        ep_camera.stop_video_stream()
        ep_robot.close()

