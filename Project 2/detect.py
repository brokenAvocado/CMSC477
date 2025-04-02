import cv2
import numpy as np

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


# hsv(134.26, 77.71%, 61.57%)

def main():
    # Open the default camera
    cam = cv2.VideoCapture(0)

    detector = Detect()
    detector.set_lower_mask(114, .2, 0)
    detector.set_upper_mask(154, 1, 1)

    while True:
        ret, frame = cam.read()

        edges = detector.edges(frame)

        center = detector.center(edges)

        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if center:
            print("Center:", center)
            cv2.circle(edges_bgr, center, 5, (0, 0, 255), 5)  # Draw center in red

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