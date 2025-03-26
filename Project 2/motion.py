import time
import robomaster
from robomaster import robot

class motion:
    def __init__(self):


        robomaster.config.ROBOT_IP_STR = "192.168.50.121"
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")
        self.ep_arm = self.ep_robot.robotic_arm
        self.ep_gripper = self.ep_robot.gripper
        self.ep_chassis = self.ep_robot.chassis

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
        self.arm_raise()

    # lower release raise
    def lrr(self):
        self.arm_lower()
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

if __name__ == '__main__':
    r1 = motion()
    r1.ep_arm.sub_position(freq=5, callback=r1.arm_position_reader)

    r1.gripper_open()

    r1.lgr()
    time.sleep(1)
    r1.move_away()



    r1.ep_arm.unsub_position()
    r1.ep_robot.close()