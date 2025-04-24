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
        self.ep_servo = self.ep_robot.servo
        
        # Camera Init
        self.ep_camera = self.ep_robot.camera

        # State Init
        self.stowed = True

    def printStatement(self,pos_info):
        x, y, z = pos_info
        print(f"Pos X: {x} Pos Y: {y} Pos Z: {z}")

    def get_robotPosition(self):
        '''
        Gets the relative position of the robot based on where it started
        '''

        self.ep_chassis.sub_position(freq = 5, callback = self.printStatement)

    def read_joint_angles(self):
        angle1 = self.ep_servo.get_angle(1)
        angle2 = self.ep_servo.get_angle(2)
        print('1: {0}   2: {1}'.format(angle1, angle2))

    def stow_arm(self): # Stows the arm back to its original position
        print('stowing')
        self.ep_arm.moveto(1, -10).wait_for_completed()
        self.ep_arm.moveto(2, 15).wait_for_completed()
        print('stowed')
        # self.ep_gripper.open(100)

    def ready_arm(self): # Readies the arm to approach an object
        print('getting ready')
        self.ep_arm.move(x=0, y=-30).wait_for_completed()
        time.sleep(1)
        print('ready')

    def pickup(self): # Picks up an object (lowers arm slightly, grips, raises)
        self.ep_gripper.open(100)
        time.sleep(1)
        self.ep_gripper.close(100)
        time.sleep(1)
        self.ep_gripper.close(100)
        time.sleep(2)
        self.ep_arm.move(x=0, y=30).wait_for_completed()
        time.sleep(1)
        self.ep_gripper.open(100)

    def grasp(self):
        self.ep_arm.move(x=-50, y=0).wait_for_completed()

    def release(self):
        self.ep_arm.move(x=-50, y=0).wait_for_completed()

    def orbit(self, velx=0, vely=0, velz=0):
        # self.ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.02)
        self.ep_chassis.move(x=0, y=0, z=15, z_speed=45).wait_for_completed()