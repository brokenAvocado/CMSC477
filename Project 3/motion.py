import pupil_apriltags
import cv2
import numpy as np
import time
import traceback
import math
import random

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

        # Arm State Init
        self.stowed = True
        self.gripper = 'opened'

        # Robot State Initz
        self.globalOffset = [0.72, 0.8, 0]
        # self.globalOffset = [0,0,0]
        self.globalPose = [0,0,0]
        self.yawOffset = 0
        self.yawInit = False
        self.orientationSet = False

        self.lastTargetZ = -90

    def updatePosition(self, pos_info):
        x, y, z = pos_info
        temp_globalX = x*np.cos(np.pi/180*self.yawOffset) - y*np.sin(np.pi/180*self.yawOffset)
        temp_globalY = -(y*np.cos(np.pi/180*self.yawOffset) + x*np.sin(np.pi/180*self.yawOffset))

        self.globalPose[0] = self.globalOffset[0] + temp_globalX
        self.globalPose[1] = self.globalOffset[1] + temp_globalY
        
    def updateRotation(self, rot_info):
        yaw, _, _= rot_info
        if not self.yawInit:
            self.yawOffset = -yaw
            self.yawInit = True    
        self.globalPose[2] = -yaw - self.yawOffset - 90
        # print(f"Raw yaw: {yaw}     GlobalPose2: {self.globalPose[2]}")
        # if yaw > -180 and yaw < 86:
            # fixedYaw = self.map_range(yaw, -180, -86, -86, -180)
            #print(f"Raw yaw: {yaw}     GlobalPose2: {self.globalPose[2]}     FixedYaw: {fixedYaw}")
            # self.globalPose[2] = fixedYaw
        # elif yaw > -86 and yaw < 180:
            # fixedYaw = self.map_range(yaw, -86, 180, 180, -86)
            #print(f"Raw yaw: {yaw}     GlobalPose2: {self.globalPose[2]}     FixedYaw: {fixedYaw}")
            # self.globalPose[2] = fixedYaw

    def map_range(self, x, in_min, in_max, out_min, out_max):
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
            
    def printStatement(self,pos_info):
        x, y, z = pos_info
        print(f"Pos X: {x} Pos Y: {y} Pos Z: {z}")

    def get_robotPosition(self):
        '''
        PUT THIS OUTSIDE WHILE LOOP
        Gets the relative position of the robot based on where it started
        '''
        self.ep_chassis.sub_position(freq = 10, callback = self.updatePosition)

    def get_robotAngle(self):
        '''
        PUT THIS OUTSIDE WHILE LOOP
        Gets the relative attitude of the robot based on where it started
        '''
        self.ep_chassis.sub_attitude(freq = 10, callback = self.updateRotation)

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
        self.ep_arm.moveto(82, 41).wait_for_completed(timeout=3)
        self.stowed = True

    def ready_arm(self): # Readies the arm to approach an object
        print("ARM LOWER")
        self.ep_arm.moveto(185, -75).wait_for_completed(timeout=3)
        self.ep_gripper.open(100)
        self.stowed = False
        print("ARM DONE")

    def pickup(self): # Picks up an object (lowers arm slightly, grips, raises)
        self.grasp()
        time.sleep(1)
        self.ep_arm.moveto(182, -38).wait_for_completed(timeout=3)

    def grasp(self):
        self.ep_gripper.close(100)
        time.sleep(1)
        self.ep_gripper.close(100)
        time.sleep(1)

    def release(self):
        self.ep_gripper.open(100)
        self.ep_gripper.open(100)
        self.ep_gripper.open(100)

    def arctan2Test(self, target_x, target_y):
        '''
        Not part of the main script
        '''
        print(180/np.pi*np.arctan2(target_x, target_y))

    def go_to(self, target_x, target_y, flipTol = 0.5, speed_mult = 0.4, speed_offset=0.05):
        '''
        Takes global x and y target positions and converts it to relative
        kinematic commands
        '''
        current_x = self.globalPose[0]
        current_y = self.globalPose[1]

        # Distances in the global frame
        dist_x = target_x-current_x 
        dist_y = target_y-current_y 

        target_z = -180/np.pi*np.arctan2(dist_x,dist_y)

        # if dist_y == 0 and dist_x > 0:
        #     target_z = -90
        # elif dist_y == 0 and dist_x < 0:
        #     target_z = 90
        # else:
        #     if dist_x > 0:
        #         target_z = -target_z 
        
        dist_rel = ((dist_x)**2+(dist_y)**2)**0.5
        vel_z = self.rotate_to(target_z)
        vel_x = speed_mult*np.arctan(dist_rel)+speed_offset

        self.ep_chassis.drive_speed(x=vel_x, y=0, z=vel_z, timeout=0.1)

        #print(f"Goal: {target_z}, Current: {self.globalPose[2]}, CurrentVel: {vel_z}")

        if abs(dist_rel) < 0.1:
            vel_x = 0
            return True
        else:
            return False
        
    def sequence(self, targets):
        '''
        Takes an array of global positional points that the robot needs to go,
        Sequentially runs each point that it needs to go to
        '''
        pos = targets[0]
        moveOn = self.go_to(pos[0], pos[1])
        if moveOn:
            targets.pop(0)
        return targets

    def rotate_to(self, target_z, rotate_tol = 0.3, speedMult = 0.05, direction = -1, flipTol = 1):
        '''
        Takes global orientation angle and will point in that direction
        '''
        diff = (target_z - self.globalPose[2] + 180) % 360 - 180
        direction = -1 if diff > 0 else 1
        if False:
            2

        if abs(diff) > rotate_tol:
            return direction*speedMult*(diff)**2
        else:
            return 0

    def avoid_tag(self, obstacles, front_distance=0.6, window_width=0.6, offset_distance=0.4):
        """
        Avoid ArUco tags by checking if they're in front, and move to the
        side opposite the robot's lateral position relative to the tag.

        Returns:
            (bool, (x, y)) — whether to move, and coordinates to move to
        """
        x, y, angle_deg = self.globalPose
        theta = math.radians(angle_deg + 90)

        # Robot heading vector
        heading_x = math.cos(theta)
        heading_y = math.sin(theta)

        # Perpendicular vector to the left
        left_x = -math.sin(theta)
        left_y = math.cos(theta)

        for tag_id, (ox, oy) in obstacles.items():
            dx = ox - x
            dy = oy - y

            # Distance from robot to tag in robot's heading and lateral directions
            forward_dist = dx * heading_x + dy * heading_y
            lateral_dist = dx * left_x + dy * left_y

            # print(f"[DEBUG] Tag {tag_id}: forward={forward_dist:.2f}, lateral={lateral_dist:.2f}")

            if 0 <= forward_dist <= front_distance and abs(lateral_dist) <= window_width / 2:
                # Unit vector from robot to tag
                tag_dx = ox - x
                tag_dy = oy - y
                tag_dist = math.hypot(tag_dx, tag_dy)
                dir_x = tag_dx / tag_dist
                dir_y = tag_dy / tag_dist

                # Perpendicular direction (left of robot-to-tag vector)
                perp_left_x = -dir_y
                perp_left_y = dir_x

                # Decide direction to move based on whether robot is to left or right of tag
                sign = -1 if lateral_dist < 0 else 1  # if robot is right of tag, move right (+1)

                move_to_x = x + sign * perp_left_x * offset_distance
                move_to_y = y + sign * perp_left_y * offset_distance

                print(f"  >> Tag {tag_id} ahead. Robot is {'left' if lateral_dist > 0 else 'right'} of tag. Moving to ({move_to_x:.2f}, {move_to_y:.2f})")
                return True, [move_to_x, move_to_y]

        return False, None

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def align_with_closet(self, closet_coordinates):
        while True:
            bot_x = self.globalPose[0]
            bot_y = self.globalPose[1]
            closet_x = closet_coordinates[0]
            closet_y = closet_coordinates[1]

            angle = math.degrees(math.atan2(closet_y - bot_y, closet_x - bot_x)) - 90

            vel_z = self.rotate_to(angle)

            if vel_z == 0:
                return
            else:
                self.ep_chassis.drive_speed(x=0, y=0, z=vel_z, timeout=0.1)

            

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