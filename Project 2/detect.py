import cv2
import numpy as np

class Detect:
    def __init__(self):
        self.mask = None
        self.FRAME_CENTER_X = None
        self.FRAME_CENTER_Y = None
        self.object_center = None

        
        # Color presets (HUE: 0, 180) (SAT: 0, 255) (VAL: 0, 255)
        self.BRICK_GREEN = [[57, 51, 0], [77, 255, 255]]
        self.BRICK_RED = [[-10, 120, 0], [10, 255, 255]]

        self.PAPER_PINK = None
        self.PAPER_ORANGE = [[5, 100, 200], [25, 255, 255]]

        # Constants
        self.LOWER = 0
        self.UPPER = 1
        self.HUE = 0
        self.SAT = 1
        self.VAL = 2

        # Moving Average Content
        self.AVG_ORIENTATION = [False] * 101
        self.IDX_ORIENTATION = 0

### Isolating the Brick ###
    # Convert BGR image to HSV
    def BGRtoHSV(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask with the lower and upper thresholds
    def mask_image(self, img, color):
        lower = color[self.LOWER]
        upper = color[self.UPPER]

        if lower[self.HUE] >= 0:
            mask = cv2.inRange(img, np.array(lower), np.array(upper))
        else:
            # -HUE to 0
            lower1 = np.array([179 + lower[self.HUE], lower[self.SAT], lower[self.VAL]])
            upper1 = np.array([179, upper[self.SAT], upper[self.VAL]])

            # 0 to HUE
            lower2 = np.array([0, lower[self.SAT], lower[self.VAL]])
            upper2 = np.array([upper[self.HUE], upper[self.SAT], upper[self.VAL]])

            # Combined Mask from -HUE to HUE
            mask = cv2.bitwise_or(cv2.inRange(img, lower1, upper1), cv2.inRange(img, lower2, upper2))

        return mask

    # Take a frame and apply color mask to find object
    def detect_object(self, frame, color):
        # Convert frame to HSV
        hsv = self.BGRtoHSV(frame)

        # Mask frame
        self.mask = self.mask_image(hsv, color)
        self.mask = cv2.medianBlur(self.mask, 7)

        if self.FRAME_CENTER_X is None:
            self.FRAME_CENTER_X = len(frame[0])/2
            self.FRAME_CENTER_Y = len(frame)/2

        return self.mask
    
    # Take in found object and find its edges
    def edges(self, object):

        edges = cv2.Canny(object, threshold1=50, threshold2=150)

        # Fill in holes
        kernel = np.ones((5, 5), np.uint8)
        edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        return edges_closed
    
    # Remove all non brick detections
    def isolate_brick(self, edge_img):
        # ⚠️ Use raw edges (must be single-channel)
        contours, _ = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a black image with same shape as input edge image (converted to BGR)
        height, width = edge_img.shape[:2]
        output_img = np.zeros((height, width, 1), dtype=np.uint8)

        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # Draw the largest contour in green
            cv2.drawContours(output_img, [largest_contour], -1, (255), 2)

        return output_img
    

### Getting the center of the brick ###
    # Given edges find the center of the object
    def center(self, edges):
        white_pixels = np.column_stack(np.where(edges == 255))

        if white_pixels.size == 0:
            return None

        cy, cx = np.mean(white_pixels, axis=0).astype(int)

        self.object_center = (cx, cy)

        return (cx, cy)
    
        # Draws the center of the object on the frame
    
    def draw_center(self, edges):
        center = self.center(edges)

        if len(edges.shape) == 2 or edges.shape[2] == 1:
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        else:
            edges_bgr = edges.copy()


        if center is not None:
            cv2.circle(edges_bgr, center, 5, (0, 0, 255), -1)

        return edges_bgr
    
    
### Getting distance to brick ###    
    # Given edges find the left and right sides of the object
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
    
    # Find length of a line (x1, y1) (x2, y2)
    def line_length(self, lines):
        lengths = []
        for x1, y1, x2, y2 in lines:
            length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            lengths.append(length)
        return lengths
    
    # Find distance from robot to object based on side lengths
    def distance_lines(self, lineLengths):
        avg = np.mean(np.array(lineLengths))
        # print(avg)
        distance = 2.156*np.exp(-0.01828*avg)
        return distance
    
    # Find distance from robot to object based on the area of the detected zone
    def distance_area_far(self, brick):
        contour, _ = cv2.findContours(brick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contour:
            A = cv2.contourArea(contour[0], oriented=False)
            output = 0.8506*np.exp(-2.033*10**-4*A)
        return output
    
    def distance_area_near(self, brick):
        contour, _ = cv2.findContours(brick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contour:
            A = cv2.contourArea(contour[0], oriented=False)
            output = -1.845*10**-4*A+1.401
        return output

    # Draws the center of the object on the frame
    def draw_center(self, edges):
        center = self.center(edges)

        if len(edges.shape) == 2 or edges.shape[2] == 1:
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        else:
            edges_bgr = edges.copy()


### Getting orientation of the brick relative to the robot ###
    def orientation(self, img, left_thresh=15, right_thresh=15, up_down_window=2):
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Get coordinates of all pixels with grayscale > 200
        bright_pixels = np.column_stack(np.where(img > 200))

        if bright_pixels.size == 0:
            return False

        # Step 1: Find the pixel closest to the bottom (max Y)
        bottommost_pixel = bright_pixels[np.argmax(bright_pixels[:, 0])]
        y = bottommost_pixel[0] - 1

        # Step 2: Get all x-positions at that y-level that are also bright
        bright_xs_at_y = np.where(img[y, :] > 200)[0]

        if bright_xs_at_y.size == 0:
            return False  # Safety check

        # Step 3: Take the average x
        x = int(np.mean(bright_xs_at_y))

        height, width = img.shape[:2]

        # Vertical range
        y_start = max(y - up_down_window, 0)
        y_end = min(y + up_down_window, height - 1)

        # Left region (1x3)
        x_left = max(x - left_thresh, 0)
        left_region = img[y_start:y_end + 1, x_left:x_left + 1]

        # Right region (1x3)
        x_right = min(x + right_thresh, width - 1)
        right_region = img[y_start:y_end + 1, x_right:x_right + 1]

        # Draw red pixels on left region
        for yy in range(y_start, y_end + 1):
            img_color[yy, x_left] = (0, 0, 255)  # Red

        # Draw red pixels on right region
        for yy in range(y_start, y_end + 1):
            img_color[yy, x_right] = (0, 0, 255)  # Red

        l = np.any(left_region > 200)
        r = np.any(right_region > 200)
        has_bright_neighbor = l or r

        # Display debug image
        cv2.imshow("Orientation Check", img_color)

        return has_bright_neighbor



    
    def orientation_avg(self, img, left_thresh=20, right_thresh=20, up_down_window=1):
        orient = self.orientation(img, left_thresh, right_thresh, up_down_window)

        self.AVG_ORIENTATION[self.IDX_ORIENTATION] = orient
        self.IDX_ORIENTATION += 1
        if self.IDX_ORIENTATION >= len(self.AVG_ORIENTATION):
            self.IDX_ORIENTATION = 0

        threshold = 2/3
        return sum(self.AVG_ORIENTATION) >= threshold * len(self.AVG_ORIENTATION)




# hsv(134.26, 77.71%, 61.57%)

def main():
    cam = cv2.VideoCapture(0)

    detector = Detect()

    while True:
        _, frame = cam.read()

        object = detector.detect_object(frame, detector.BRICK_GREEN)

        edges = detector.edges(object)

        brick = detector.isolate_brick(edges)

        aligned = detector.orientation_avg(brick)

        print(f'AVG: {aligned}')

        # Display the captured frame
        #cv2.imshow('Camera', edges)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()