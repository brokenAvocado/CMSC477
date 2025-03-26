import time
import robomaster
from robomaster import robot

def gripper_close(power=100):
    ep_gripper.close(power)
    time.sleep(1)
    ep_gripper.pause()

def gripper_open(power=100):
    ep_gripper.open(power)
    time.sleep(1)
    ep_gripper.pause()

if __name__ == '__main__':
    robomaster.config.ROBOT_IP_STR = "192.168.50.121"
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")

    ep_gripper = ep_robot.gripper

    gripper_open()
    gripper_close()

    ep_robot.close()