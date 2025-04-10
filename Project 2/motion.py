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
import threading

class AprilTagDetector: # Given
    def __init__(self, family="tag36h11", threads=2, marker_size_m=0.16):
        K = np.array([[314, 0, 320], [0, 314, 180], [0, 0, 1]]) # Camera focal length and center pixel
        marker_size_m = 0.153 # Size of the AprilTag in meters
        self.camera_params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        self.marker_size_m = marker_size_m
        self.detector = pupil_apriltags.Detector(family, threads)

    def find_tags(self, frame_gray):
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections

    def draw_detections(self, frame, detections): # Given
        for detection in detections:
            pts = detection.corners.reshape((-1, 1, 2)).astype(np.int32)

            frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

            top_left = tuple(pts[0][0])  # First corner
            top_right = tuple(pts[1][0])  # Second corner
            bottom_right = tuple(pts[2][0])  # Third corner
            bottom_left = tuple(pts[3][0])  # Fourth corner
            cv2.line(frame, top_left, bottom_right, color=(0, 0, 255), thickness=2)
            cv2.line(frame, top_right, bottom_left, color=(0, 0, 255), thickness=2)

    def get_pose_apriltag_in_camera_frame(self, detection):
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

class motion:
    def __init__(self):
        robomaster.config.ROBOT_IP_STR = "192.168.50.121"
        
        # Robot Init
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")
        self.ep_arm = self.ep_robot.robotic_arm
        self.ep_gripper = self.ep_robot.gripper
        self.ep_chassis = self.ep_robot.chassis
        
        # Camera Init
        self.ep_camera = self.ep_robot.camera

        # State Booleans
        self.isLost = True
        self.isGrip = False

    def gripper_close(self, power=100):
        self.ep_gripper.close(power)
        time.sleep(1)
        self.ep_gripper.pause()

    def gripper_open(self, power=100):
        self.ep_gripper.open(power)
        time.sleep(1)
        self.ep_gripper.pause()

    def arm_forward(self):
        self.ep_arm.move(x=50, y=0).wait_for_completed()

    def arm_backward(self):
        self.ep_arm.move(x=-50, y=0).wait_for_completed()

    def arm_lower(self):
        self.ep_arm.move(x=0, y=-50).wait_for_completed()

    def arm_raise(self):
        self.ep_arm.move(x=0, y=50).wait_for_completed()

    def arm_position_reader(self, sub_info):
        pos_x, pos_y = sub_info
        print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))

    # lower grab raise
    def lgr(self):
        # self.arm_lower()
        time.sleep(1)
        self.gripper_close()
        time.sleep(1)
        self.gripper_close()
        time.sleep(1)
        self.arm_raise()

    # lower release raise
    def lrr(self):
        self.arm_lower()
        time.sleep(1)
        self.gripper_open()
        time.sleep(1)
        self.gripper_open()
        time.sleep(1)
        # self.arm_raise()

    # move backwards -> rotate 90 -> drop block -> move backwards a little -> rotate -90
    def move_away(self):
        self.ep_chassis.drive_speed(x=-0.3, y=0, z=0, timeout=5)
        time.sleep(2)
        self.ep_chassis.drive_speed(x=0, y=0, z=90, timeout=5)
        time.sleep(2)
        self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(1)
        self.lrr()
        self.ep_chassis.drive_speed(x=-0.1, y=0, z=0, timeout=5)
        time.sleep(1)
        self.ep_chassis.drive_speed(x=0, y=0, z=-90, timeout=5)
        time.sleep(2)

    def scan(self):
        self.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

    def move_to_coarse(self, TPose, Rpose):
        # AprilTag Parameters
        # Px = 2
        # Py = Px
        # Pz = 300
        # offsetX = 0.6
        # offsetY = 0

        # Color Masking Parameters
        Px = 1.3
        Py = 0.005
        Pz = 300
        offsetX = 0.32
        offsetY = 0

        errorX = TPose[2]-offsetX
        errorY = TPose[0]-offsetY

        velx = Px*(errorX)
        vely = Py*(errorY)
        velz = Pz*(Rpose[1])

        if not TPose[2]:
            velx = 0

        self.ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.02)
        return errorX, errorY

    def move_to_fine(self):
        self.ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=10)
        time.sleep(3.2)
        self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.02)
        # self.ep_chassis.move(x=0.2, y=0, z=0, xy_speed = 1)
        self.lgr()
        time.sleep(1)
        self.move_away()
        self.isGrip = False
