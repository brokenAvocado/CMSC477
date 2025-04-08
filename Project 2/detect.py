import cv2
import numpy as np

class Detect:
    def __init__(self):
        self.lower = None
        self.upper = None
        self.mask = None
        self.FRAME_CENTER_X = None
        self.FRAME_CENTER_Y = None

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

        self.FRAME_CENTER_X = len(frame[0])/2
        self.FRAME_CENTER_Y = len(frame)/2

        return output
    
    def edges(self, frame):
        # Step 1: Detect green blob
        green_regions = self.detect_object(frame)

        # Step 2: Convert to grayscale
        gray = cv2.cvtColor(green_regions, cv2.COLOR_BGR2GRAY)

        # Step 3: Apply Canny edge detection
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        return edges
    
    def center(self, edges, tol=15):
        # Get coordinates of all white pixels (value 255)
        white_pixels = np.column_stack(np.where(edges == 255))

        if white_pixels.size < tol:
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
    
    def distance(self, lineLengths):
        avg = np.mean(np.array(lineLengths))
        # print(avg)
        distance = 2.156*np.exp(-0.01828*avg)
        return distance


# hsv(134.26, 77.71%, 61.57%)

def main():
    # Open the default camera
    cam = cv2.VideoCapture(0)

    detector = Detect()
    # Green Values
    # detector.set_lower_mask(114, .2, 0)
    # detector.set_upper_mask(154, 1, 1)

    # Red Values
    detector.set_lower_mask(0, .760, 0)
    detector.set_upper_mask(21, 1, 1)

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

            #print(detector.line_length(vertical_lines))

        

        # Display the captured frame
        cv2.imshow('Camera', edges_bgr)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()