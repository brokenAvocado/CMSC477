import robomaster
from robomaster import robot
import keyboard
import time

# Movement parameters
MOVE_DIST = 0.2  # meters
ROTATE_ANGLE = 15  # degrees
MOVE_SPEED = 0.7  # m/s
ROTATE_SPEED = 45  # deg/s

# Arm movement parameters
ARM_STEP = 10  # degrees per press

def main():
    robomaster.config.ROBOT_IP_STR = "192.168.50.121"
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")

    ep_chassis = ep_robot.chassis

    try:
        print("Use W/A/S/D to move, Q/E to rotate, Up/Down to control arm. ESC to quit.")
        
        while True:
            if keyboard.is_pressed("w"):
                ep_chassis.move(x=MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("s"):
                ep_chassis.move(x=-MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("a"):
                ep_chassis.move(x=0, y=-MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("d"):
                ep_chassis.move(x=0, y=MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("q"):
                ep_chassis.move(x=0, y=0, z=ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("e"):
                ep_chassis.move(x=0, y=0, z=-ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("up"):
                ep_robot.gripper.move(arm=-ARM_STEP).wait_for_completed()  # Replace with correct API
            elif keyboard.is_pressed("down"):
                ep_robot.gripper.move(arm=ARM_STEP).wait_for_completed()   # Replace with correct API
            elif keyboard.is_pressed("esc"):
                print("Exiting control...")
                break
            time.sleep(0.1)

    finally:
        ep_robot.close()

if __name__ == "__main__":
    main()
