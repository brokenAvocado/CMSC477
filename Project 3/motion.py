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
import keyboard

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
        self.gripper = 'opened'

    def printStatement(self,pos_info):
        x, y, z = pos_info
        print(f"Pos X: {x} Pos Y: {y} Pos Z: {z}")

    def get_robotPosition(self):
        '''
        Gets the relative position of the robot based on where it started
        '''

        self.ep_chassis.sub_position(freq = 5, callback = self.printStatement)

    def get_arm_position(self):
        self.ep_arm.sub_position(freq=5, callback=self.arm_position_callback)

    def arm_position_callback(_, positions):
        pos_x, pos_y = (positions[0], positions[1])
        if pos_y > 4000000000:
            pos_y = pos_y - 2**32
        print(f"Position X: {pos_x}, Position Y: {pos_y}")

    def get_gripper_status(self):
        self.ep_gripper.sub_status(freq=5, callback=self.gripper_status_callback)

    def gripper_status_callback(self, status):
        self.gripper = status
        print(self.gripper)
    
    def read_joint_angles(self):
        angle1 = self.ep_servo.get_angle(1)
        angle2 = self.ep_servo.get_angle(2)
        angle3 = self.ep_servo.get_angle(3)
        print(f'1: {angle1}   2: {angle2}   3: {angle3}')

    def stow_arm(self): # Stows the arm back to its original position
        self.ep_arm.moveto(82, 41).wait_for_completed()
        self.stowed = True

    def ready_arm(self): # Readies the arm to approach an object
        self.ep_arm.moveto(185, -75).wait_for_completed()
        self.stowed = False

    def pickup(self): # Picks up an object (lowers arm slightly, grips, raises)
        self.grasp()
        time.sleep(1)
        self.ep_arm.moveto(182, -38).wait_for_completed()

    def grasp(self):
        self.ep_gripper.close(100)
        time.sleep(1)
        self.ep_gripper.close(100)
        time.sleep(1)

    def release(self):
        self.ep_gripper.open(100)

    def orbit(self, velx=0, vely=0, velz=0):
        # self.ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.02)
        self.ep_chassis.move(x=0, y=0, z=15, z_speed=45).wait_for_completed()

    def teleop(self):
        move_speed = 0.3
        rotate_speed = 45
        # print("Use W/A/S/D to move, Q/E to rotate, Up/Down to control arm. ESC to quit.")
        if keyboard.is_pressed("w"):
            self.ep_chassis.drive_speed(x=move_speed, timeout=0.1)
        elif keyboard.is_pressed("s"):
            self.ep_chassis.drive_speed(x=-move_speed, timeout=0.1)
        # else:
        #     self.ep_chassis.drive_speed(x=0, timeout=0.1)

        elif keyboard.is_pressed("a"):
            self.ep_chassis.drive_speed(y=-move_speed, timeout=0.1)
        elif keyboard.is_pressed("d"):
            self.ep_chassis.drive_speed(y=move_speed, timeout=0.1)
        # else:
        #     self.ep_chassis.drive_speed(y=0, timeout=0.1)

        elif keyboard.is_pressed("q"):
            self.ep_chassis.drive_speed(z=-rotate_speed, timeout=0.1)
        elif keyboard.is_pressed("e"):
            self.ep_chassis.drive_speed(z=rotate_speed, timeout=0.1)
        # else:
        #     self.ep_chassis.drive_speed(z=0, timeout=0.1)
        # elif keyboard.is_pressed("up"):
        #     ep_robot.gripper.move(arm=-ARM_STEP).wait_for_completed()  # Replace with correct API
        # elif keyboard.is_pressed("down"):
        #     ep_robot.gripper.move(arm=ARM_STEP).wait_for_completed()   # Replace with correct API
        # time.sleep(0.1)
        # self.ep_robot.close()