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
        ep_camera = self.ep_robot.camera
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
    def scan(self):
        self.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)

    def move_to_coarse(self, TPose, Rpose):
        Px = 1.5
        Py = Px
        Pz = 300

        offsetX = 0.1
        offsetY = 0

        errorX = TPose[2]-offsetX
        errorY = TPose[0]-offsetY

        if(abs(errorX) < 0.02):
            errorX = 0
        if(abs(errorY) < 0.02):
            errorY = 0

        velx = Px*(errorX)
        vely = -Py*(errorY)
        velz = Pz*(Rpose[1])

        self.ep_chassis.drive_speed(x=velx, y=vely, z=velz, timeout = 0.05)
        return errorX, errorY

    def move_to_fine(self):
        velx = 0.2
        self.ep_chassis.drive_speed(x=velx, y=0, z=0, timeout = 0.5)
        time.sleep(1)
        
def aprilTagTest():
    init = False
    objects = []
    errorX = 10
    errorY = 10

    startTime = time.time()
    while True:
        try:
            img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray.astype(np.uint8)

        while (time.time()-startTime <= 6):
            r1.ep_chassis.drive_speed(x=0, y=0, z=60, timeout = 0.05)
            tags = apriltag.find_tags(gray)
            for tag in tags:
                if not(tag in objects): 
                    objects.append(tag)

        tags = apriltag.find_tags(gray)
        target = objects[0]
        if target in tags:
            pos, rot = apriltag.get_pose_apriltag_in_camera_frame(target)
            if errorX > 0.1 and errorY > 0.1:
                errorX, errorY = r1.move_to_coarse(pos, rot)
            else:
                r1.move_to_fine()


        else:
            r1.ep_chassis.drive_speed(x=0, y=0, z=60, timeout = 0.05)

        print(objects)


if __name__ == "__main__":
    # Robot Init
    r1 = motion()

    # AprilTag Init
    apriltag = AprilTagDetector()
    '''
    # Pseudo-code
    # list of sorted
    # list of real-life
    # 
    # while (real-life != sorted)
    #   Do once:
    #       Count number of objects in frame by rotating
    #   Declare pairs of objects to move
    #
    #   Detect object 1
    #   Go to object 1
    #   Grasp object 1
    #   Move to random position
    #   Drop object 1
    #
    #   Detect object 2
    #   Go to object 2
    #   Grasp object 2
    #
    #   Detect pad 1
    #   Move to proper place
    #   Drop object 2
    #
    #   (We can't do this yet without detection)
    #   Detect object 1
    #   Go to object 1
    #   Grasp object 1
    #   Detect pad 1
    #   Move to proper position
    #   Drop object 1
    #
    #   Repeat
    '''
    try:
        aprilTagTest()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        print('Waiting for robomaster shutdown')
        r1.ep_camera.stop_video_stream()
        r1.ep_robot.close()
