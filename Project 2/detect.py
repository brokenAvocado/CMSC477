import cv2
import numpy as np

class Detect:
    def __init__(self):
        self.mask = None
        self.FRAME_CENTER_X = None
        self.FRAME_CENTER_Y = None

        
        # Color presets (HUE: 0, 180) (SAT: 0, 255) (VAL: 0, 255)
        self.GREEN = [[57, 51, 0], [77, 255, 255]]
        self.RED = [[-15, 51, 0], [15, 255, 255]]

        # Constants
        self.LOWER = 0
        self.UPPER = 1
        self.HUE = 0
        self.SAT = 1
        self.VAL = 2
        

    # Convert BGR image to HSV
    def BGRtoHSV(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask with the lower and upper thresholds
    def mask_image(self, img, color):
        lower = color[self.LOWER]
        upper = color[self.UPPER]

        if lower[self.HUE] > 0:
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

        # Fill in holes
        kernel = np.ones((5, 5), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)

        output = np.zeros_like(frame)

        output[self.mask > 0] = (0, 255, 0)

        if self.FRAME_CENTER_X is None:
            self.FRAME_CENTER_X = len(frame[0])/2
            self.FRAME_CENTER_Y = len(frame)/2

        return output
    
    # Take in found object and find its edges
    def edges(self, object):
        gray = cv2.cvtColor(object, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        return edges
    
    # Given edges find the center of the object
    def center(self, edges):
        white_pixels = np.column_stack(np.where(edges == 255))

        if white_pixels.size == 0:
            return None

        cy, cx = np.mean(white_pixels, axis=0).astype(int)

        return (cx, cy)
    
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
    
    def distance(self, lineLengths):
        avg = np.mean(np.array(lineLengths))
        # print(avg)
        distance = 2.156*np.exp(-0.01828*avg)
        return distance


# hsv(134.26, 77.71%, 61.57%)

def main():
    cam = cv2.VideoCapture(0)

    detector = Detect()

    while True:
        _, frame = cam.read()

        object = detector.detect_object(frame, detector.GREEN)

        # Display the captured frame
        cv2.imshow('Camera', object)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()