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

class Detect:
    def __init__(self):
        self.lower = None
        self.upper = None
        self.mask = None

    def set_lower_mask(self, hue, sat, val):
        hue = 179.0/360.0*hue
        sat = 255.0*sat
        val = 255.0*val

        self.lower = np.array([hue, sat, val])

    def set_upper_mask(self, hue, sat, val):
        hue = 179.0/360.0*hue
        sat = 255.0*sat
        val = 255.0*val

        self.upper = np.array([hue, sat, val])

    def BGRtoHSV(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    def mask_image(self, hsv):
        mask = cv2.inRange(hsv, self.lower, self.upper)
        return mask

    def detect_object(self, frame):
        hsv = self.BGRtoHSV(frame)
        self.mask = self.mask_image(hsv)
        self.mask = cv2.medianBlur(self.mask, 7)
        kernel = np.ones((5, 5), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)

        output = np.zeros_like(frame)

        output[self.mask > 0] = (0, 255, 0)

        return output
    
    def edges(self, frame):
        # Step 1: Detect green blob
        green_regions = self.detect_object(frame)

        # Step 2: Convert to grayscale
        gray = cv2.cvtColor(green_regions, cv2.COLOR_BGR2GRAY)

        # Step 3: Apply Canny edge detection
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        return edges
    
    def center(self, edges):
        # Get coordinates of all white pixels (value 255)
        white_pixels = np.column_stack(np.where(edges == 255))

        if white_pixels.size == 0:
            return None  # No white pixels, so no edges found

        # Calculate the average (mean) x and y coordinates
        cy, cx = np.mean(white_pixels, axis=0).astype(int)  # Note: rows = y, cols = x

        return (cx, cy)
    
    def sides(self, edges, center_x, angle_thresh=75):
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180,
                                threshold=40, minLineLength=30, maxLineGap=40)

        if lines is None:
            return []

        left_group = []
        right_group = []

        # Separate vertical lines into left and right of the center
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if abs(angle) > angle_thresh:
                x_avg = (x1 + x2) / 2
                if x_avg < center_x:
                    left_group.append((x1, y1, x2, y2))
                else:
                    right_group.append((x1, y1, x2, y2))

        # Helper function to combine group
        def combine_group(group, flip=False):
            if not group:
                return None
            x1_vals = [line[0] for line in group]
            x2_vals = [line[2] for line in group]
            y_vals = [line[1] for line in group] + [line[3] for line in group]

            x1_avg = int(np.mean(x1_vals))
            x2_avg = int(np.mean(x2_vals))
            y_min = min(y_vals)
            y_max = max(y_vals)

            if flip:
                x1_avg, x2_avg = x2_avg, x1_avg  # Flip for left line

            return (x1_avg, y_max, x2_avg, y_min)

        # Combine both sides
        combined = []
        left_line = combine_group(left_group, flip=True)
        right_line = combine_group(right_group, flip=False)

        if left_line:
            combined.append(left_line)
        if right_line:
            combined.append(right_line)

        return combined
    
    def line_length(self, lines):
        lengths = []
        for x1, y1, x2, y2 in lines:
            length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            lengths.append(length)
        return lengths

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
        
# def aprilTagTest():
#     init = False
#     objects = []
#     errorX = 10
#     errorY = 10

#     startTime = time.time()
#     r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)


#     while True:
#         try:
#             img = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
#         except Empty:
#             time.sleep(0.001)
#             continue

#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         gray.astype(np.uint8)

#         tags = apriltag.find_tags(gray)

#         if (time.time()-startTime <= 12):
#             r1.ep_chassis.drive_speed(x=0, y=0, z=30, timeout = 0.05)
#             for tag in tags:
#                 if not(tag.tag_id in objects): 
#                     objects.append(tag)
#         else:
#             target = objects[0]
#             for tag in tags:
#                 if target == tag.tag_id:
#                     pos, rot = apriltag.get_pose_apriltag_in_camera_frame(target)
#                     if errorX > 0.03 and errorY > 0.03:
#                         errorX, errorY = r1.move_to_coarse(pos, rot)
#                     else:
#                         r1.move_to_fine()
#                         r1.lgr()
#                         time.sleep(1)
#                         r1.move_away()
#                 else:
#                     r1.ep_chassis.drive_speed(x=0, y=0, z=40, timeout = 0.05)

#         apriltag.draw_detections(img, tags)
#         cv2.imshow("img", img)

#         if cv2.waitKey(1) == ord('q'):
#             break

if __name__ == "__main__":
    # Robot Init
    r1 = motion()
    r1.gripper_open()

    # Detection
    cam = r1.ep_camera

    detector = Detect()
    detector.set_lower_mask(114, .2, 0)
    detector.set_upper_mask(154, 1, 1)

    while True:
        ret, frame = cam.read()
        edges = detector.edges(frame)
        center = detector.center(edges)

        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if center:
            center_x, center_y = center
            vertical_lines = detector.sides(edges, center_x)

            # Draw on color image
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            for x1, y1, x2, y2 in vertical_lines:
                cv2.line(edges_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green
            cv2.circle(edges_bgr, center, 10, (0, 0, 255), -1)  # Red dot

            cv2.imshow('Camera', edges_bgr)

            print(detector.line_length(vertical_lines))

        

        # Display the captured frame
        cv2.imshow('Camera', edges_bgr)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()

    
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
        None
        # aprilTagTest()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(traceback.format_exc())
    finally:
        r1.ep_arm.unsub_position()
        print('Waiting for robomaster shutdown')
        r1.ep_camera.stop_video_stream()
        r1.ep_robot.close()
