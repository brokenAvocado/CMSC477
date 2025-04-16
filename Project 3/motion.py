import pupil_apriltags
import cv2
import numpy as np
import time
import traceback

from queue import Empty
import time
import robomaster
from robomaster import robot
import sys

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

    def orbit(self, velx=0, vely=0, velz=0):
        # self.ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.02)
        self.ep_chassis.move(x=0, y=0, z=15, z_speed=45).wait_for_completed()