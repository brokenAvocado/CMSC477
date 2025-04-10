import cv2
import numpy as np

class Detect:
    def __init__(self):
        self.mask = None
        self.FRAME_CENTER_X = None
        self.FRAME_CENTER_Y = None
        self.object_center = None

        
        # Color presets (HUE: 0, 180) (SAT: 0, 255) (VAL: 0, 255)
        # self.GREEN = [[57, 51, 0], [77, 255, 255]]
        # self.RED = [[-10, 120, 0], [10, 255, 255]]

        self.GREEN = [[57, 153, 51], [77, 255, 255]]
        self.RED = [[-10, 194, 0], [10, 255, 255]]

        self.PAPER_ORANGE = [[5, 100, 200], [25, 255, 255]]

        # Constants
        self.LOWER = 0
        self.UPPER = 1
        self.HUE = 0
        self.SAT = 1
        self.VAL = 2

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
    def orientation(self, img, left_thresh=15, up_down_window=2):
        # Get coordinates of all pixels with grayscale > 100
        bright_pixels = np.column_stack(np.where(img > 100))

        if bright_pixels.size == 0:
            return False

        # Find the pixel closest to the bottom
        bottommost_pixel = bright_pixels[np.argmax(bright_pixels[:, 0])]
        y, x = bottommost_pixel[0], bottommost_pixel[1]

        # Coordinates for a 1x3 strip 10 pixels to the left
        x_target = max(x - left_thresh, 0)
        y_start = max(y - up_down_window, 0)
        y_end = min(y + up_down_window, img.shape[0] - 1)

        # Extract the region
        region = img[y_start:y_end + 1, x_target:x_target + 1]

        # Check if any pixel in that region exceeds brightness threshold
        has_bright_neighbor = np.any(region > 100)

        return has_bright_neighbor




# hsv(134.26, 77.71%, 61.57%)

def main():
    cam = cv2.VideoCapture(0)

    detector = Detect()

    while True:
        _, frame = cam.read()

        object = detector.detect_object(frame, detector.GREEN)

        edges = detector.edges(object)

        brick = detector.isolate_brick(edges)

        detector.orientation(brick)

        # Display the captured frame
        cv2.imshow('Camera', brick)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()