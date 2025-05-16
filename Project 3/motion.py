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

        # Arm State Init
        self.stowed = True
        self.gripper = 'opened'

        # Robot State Init
        # self.globalOffset = [0.67, 0.49, 0]
        self.globalOffset = [0,0,0]
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

    def arctan2Test(self, target_x, target_y):
        '''
        Not part of the main script
        '''
        print(180/np.pi*np.arctan2(target_x, target_y))

    def go_to(self, target_x, target_y, flipTol = 0.5, speed_mult = 0.2, speed_offset=0.05):
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
        diff = 0
        if False:
            2

        print(f'Target: {target_z}, Pose Z: {self.globalPose[2]}, Sum: {abs(target_z) + abs(self.globalPose[2])}, Sign Target: {np.sign(target_z)} Sign Pose: {np.sign(self.globalPose[2])}')
        if (abs(target_z) + abs(self.globalPose[2]) > 180) and (np.sign(target_z) != np.sign(self.globalPose[2])):
            diff = target_z + self.globalPose[2]
            direction = np.sign(abs(target_z)-abs(self.globalPose[2])) 
        # elif (np.sign(target_z) != np.sign(self.globalPose[2])):
        #     diff = target_z-self.globalPose[2]
        #     if diff > 0:
        #         direction = 1 # Rotate clockwise
        else:
            diff = target_z-self.globalPose[2]
            if diff < 0:
                direction = 1

        if direction == 1:
            print(f'Direction: Clockwise,         Speed: {abs(diff)}')
        else:
            print(f'Direction: Counter Clockwise, Speed: {abs(diff)}')

        if abs(diff) > rotate_tol:
            return direction*speedMult*(diff)**2
        else:
            return 0
            
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