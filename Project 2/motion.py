import time
import robomaster
from robomaster import robot

class motion:
    def __init__(self):


        robomaster.config.ROBOT_IP_STR = "192.168.50.121"
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")
        self.ep_gripper = self.ep_robot.gripper

    def gripper_close(self, power=100):
        self.ep_gripper.close(power)
        time.sleep(1)
        self.ep_gripper.pause()

    def gripper_open(self, power=100):
        self.ep_gripper.open(power)
        time.sleep(1)
        self.ep_gripper.pause()