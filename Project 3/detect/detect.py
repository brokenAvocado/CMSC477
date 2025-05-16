import cv2
import os
import random
import shutil
import time
from ultralytics import YOLO
from queue import Empty
import shutil
import re
import numpy as np
from itertools import combinations
from sklearn.cluster import KMeans


class Detector:
    def __init__(self, sample_frame, ep_chassis):
        self.cone_model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\detect\\cone_model\\weights\\best.pt")
        self.bot_model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\detect\\bot_model\\weights\\best.pt")
        self.closet_brick_model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\detect\\closet_brick_model\\weights\\best.pt")

        self.ep_chassis = ep_chassis

        self.frame_shape = sample_frame.shape
        return
    

    def get_cones_simple(self, frame, model=None):
        if model == None:
            model = self.cone_model

        # Grayscaling
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)

        # Process through model
        if model.predictor:
            model.predictor.args.verbose = False
        result = model.predict(source=gray_bgr, show=False)[0]

        boxes = result.boxes
        for box in boxes:
            xyxy = box.xyxy.cpu().numpy().flatten()
            cv2.rectangle(frame,
                          (int(xyxy[0]), int(xyxy[1])), 
                          (int(xyxy[2]), int(xyxy[3])),
                           color=(0, 0, 255), thickness=2)
            
        cv2.imshow('frame', frame)

    def get_cones_no_merge(self, frame, model=None):
        if model == None:
            model = self.cone_model

        # Grayscaling
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)

        # Process through model
        if model.predictor:
            model.predictor.args.verbose = False
        result = model.predict(source=gray_bgr, show=False)[0]

        # Get raw boxes
        boxes = result.boxes
        raw_boxes = [box.xyxy.cpu().numpy().astype(int).flatten() for box in boxes]

        # === Orange Pixel Filtering ===
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for orange (can be tuned)
        lower_orange = np.array([3, 45, 160])
        upper_orange = np.array([12, 255, 255])

        final_boxes = []
        for box in raw_boxes:
            x1, y1, x2, y2 = box
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

            roi = hsv_frame[y1:y2, x1:x2]
            mask = cv2.inRange(roi, lower_orange, upper_orange)
            orange_pixel_count = cv2.countNonZero(mask)

            if orange_pixel_count > 20:  # Threshold (tune as needed)
                final_boxes.append(box)

        # Get and sort box centers
        box_centers = []
        for box in final_boxes:
            x1, y1, x2, y2 = box
            box_centers.append(((x1 + x2) / 2, (y1 + y2) / 2))

        box_centers_sorted = sorted(box_centers, key=lambda x: x[0])
        return final_boxes, box_centers_sorted

    # Processing Functions #
    # Cones Detection/Triangulation
    def get_cones(self, frame, model=None):
        if model == None:
            model = self.cone_model

        # Grayscaling
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)

        # Process through model
        if model.predictor:
            model.predictor.args.verbose = False
        result = model.predict(source=gray_bgr, show=False)[0]

        # Find and merge boxes
        boxes = result.boxes
        threshold = 15 # Euclidean Distance to merge
        raw_boxes = [box.xyxy.cpu().numpy().flatten() for box in boxes]
        centers = [((b[0] + b[2]) / 2, (b[1] + b[3]) / 2) for b in raw_boxes]

        used = set()
        merged_boxes = []

        for i, j in combinations(range(len(raw_boxes)), 2):
            if i in used or j in used:
                continue
            c1, c2 = centers[i], centers[j]
            dist = np.linalg.norm(np.array(c1) - np.array(c2))
            if dist < threshold:
                merged = (np.array(raw_boxes[i]) + np.array(raw_boxes[j])) / 2
                merged_boxes.append(merged.astype(int))
                used.update([i, j])

        for idx, box in enumerate(raw_boxes):
            if idx not in used:
                merged_boxes.append(box.astype(int))

        # === Orange Pixel Filtering ===
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for orange (can be tuned)
        lower_orange = np.array([3, 45, 160])
        upper_orange = np.array([12, 255, 255])

        final_boxes = []
        for box in merged_boxes:
            x1, y1, x2, y2 = box
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

            roi = hsv_frame[y1:y2, x1:x2]
            mask = cv2.inRange(roi, lower_orange, upper_orange)
            orange_pixel_count = cv2.countNonZero(mask)

            if orange_pixel_count > 20:  # Threshold (tune as needed)
                final_boxes.append(box)

        # Draw boxes
        box_centers = []
        for box in final_boxes:
            x1, y1, x2, y2 = box
            box_centers.append(((x1 + x2) / 2, (y1 + y2) / 2))

        box_centers_sorted = sorted(box_centers, key=lambda x: x[0])
        return final_boxes, box_centers_sorted
    
    def group_slopes(self, box_centers, slope_threshold=0.1):
        if len(box_centers) < 2:
            print("Not enough points to group.")
            return box_centers, []
        
        # Convert and sort box_centers by x
        points = np.array(box_centers)
        points = points[np.argsort(points[:, 0])]

        # Calculate slopes between successive pairs
        slopes = []
        pair_indices = []

        for i in range(len(points) - 1):
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            slope = float('inf') if dx == 0 else dy / dx
            slopes.append([slope])  # For KMeans
            pair_indices.append((i, i+1))  # Store index pairs

        if len(slopes) < 2:
            print("Not enough points to group.")
            return [tuple(pt) for pt in points], []

        # KMeans clustering
        kmeans = KMeans(n_clusters=2, n_init=10, random_state=42)
        labels = kmeans.fit_predict(slopes)

        # Create index sets from pair clusters
        group1_idx = set()
        group2_idx = set()

        for i, (idx1, idx2) in enumerate(pair_indices):
            if labels[i] == 0:
                group1_idx.update([idx1, idx2])
            else:
                group2_idx.update([idx1, idx2])

        # Check if average slopes are close; merge if so
        group1_slopes = [slopes[i][0] for i in range(len(slopes)) if labels[i] == 0]
        group2_slopes = [slopes[i][0] for i in range(len(slopes)) if labels[i] == 1]
        avg1 = np.mean(group1_slopes)
        avg2 = np.mean(group2_slopes)

        if abs(avg1 - avg2) < slope_threshold:
            return list(map(tuple, points)), []

        # Find intersection so groups share one box center
        shared_idx = group1_idx & group2_idx
        if not shared_idx:
            # If no natural shared point, force one (middle of sequence)
            shared_idx = {pair_indices[len(pair_indices)//2][1]}
            group1_idx = set(range(min(shared_idx)+1))
            group2_idx = set(range(min(shared_idx), len(points)))

        # Sort the group indices and convert back to coordinates
        group1 = [tuple(points[i]) for i in sorted(group1_idx)]
        group2 = [tuple(points[i]) for i in sorted(group2_idx)]

        return group1, group2
    
    def get_corners(self, group1, group2):
        corners = list(set(group1) & set(group2))
        return corners
    
    # Bot Detection #
    def get_bots(self, frame, model=None):
        if model == None:
            model = self.bot_model

        # Run YOLO prediction
        results = model.predict(source=frame, show=False)
        boxes = []

        # Extract boxes from the first result
        if results and len(results) > 0:
            result = results[0]
            if hasattr(result, 'boxes') and result.boxes is not None:
                for box in result.boxes:
                    # Get [x1, y1, x2, y2] as integers
                    xyxy = box.xyxy.cpu().numpy().flatten().astype(int)
                    boxes.append(tuple(xyxy))  # Add as tuple (x1, y1, x2, y2)

        return boxes

    # Brick Detection #
    def get_closet_bricks(self, frame, model=None):
        if model == None:
            model = self.cone_model

        # Run YOLO prediction
        results = model.predict(source=frame, show=False)
        boxes = []

        # Extract boxes from the first result
        if results and len(results) > 0:
            result = results[0]
            if hasattr(result, 'boxes') and result.boxes is not None:
                for box in result.boxes:
                    # Get [x1, y1, x2, y2] as integers
                    xyxy = box.xyxy.cpu().numpy().flatten().astype(int)
                    boxes.append(tuple(xyxy))  # Add as tuple (x1, y1, x2, y2)

        return boxes

    def closest_brick(self, boxes):
        if not boxes:
            return None
        
        frame_shape = self.frame_shape

        frame_height, frame_width = frame_shape[:2]
        bottom_center = (frame_width // 2, frame_height)  # (x, y) at bottom center

        min_distance = float('inf')
        closest = None

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten().astype(int)
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Compute Euclidean distance to bottom center
            dist = np.sqrt((center_x - bottom_center[0])**2 + (center_y - bottom_center[1])**2)

            if dist < min_distance:
                min_distance = dist
                closest = (x1, y1, x2, y2)

        cx = (closest[0] + closest[2]) // 2
        cy = (closest[1] + closest[3]) // 2

        return cx, cy
    
    def align_to_brick(self, cx, threshold=10):
        frame_width = self.frame_shape[1]
        center_x = frame_width // 2
        offset = cx - center_x

        if abs(offset) < threshold:
            print("Brick is centered.")
            return True# No turn needed

        if offset < 0:
            print("Brick is left — turning left")
            # chassis.rotate(angle=-5)  # Turn left 5 degrees
            self.ep_chassis.drive_speed(x=0, y=0, z=-5, timeout=0.1)
        else:
            print("Brick is right — turning right")
            self.ep_chassis.drive_speed(x=0, y=0, z=5, timeout=0.1)

    def approach(self, cy, frame_height, tolerance=200):
        frame_height = self.frame_shape[0]

        # Initialize robot if needed
        bottom_threshold = frame_height - tolerance

        if cy < bottom_threshold:
            print(f"Brick is not close enough (cy = {cy}) — driving forward.")
            self.ep_chassis.drive_speed(x=0.05, y=0, z=0)

            return False
        else:
            print(f"Brick is within approach threshold (cy = {cy}) — stopping.")

            return True

    # Draw Functions #
    def draw_lines_between_pairs(self, frame, box_centers):
        # Convert to integer pixel coords
        points = [tuple(map(int, pt)) for pt in box_centers]

        # Draw lines between successive pairs
        for i in range(len(points) - 1):
            cv2.line(frame, points[i], points[i + 1], color=(0, 255, 0), thickness=2)

        return frame

    def draw_cone_corners(self, frame, corners):
        for corner in corners:
            x, y = int(corner[0]), int(corner[1])
            cv2.circle(frame, (x, y), radius=10, color=(255, 0, 0), thickness=2)


    # Brick Detection Control Loop #
    # BEFORE RUNNING THIS THE GRIPPER MUST BE LOWERED AND THE BOT MUST BE FACING THE CLOSET
    def run_closet_bricks(self, ep_camera):
        while True:
            try:
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            except Empty:
                time.sleep(0.001)
                continue

            brick_boxes = self.get_closet_bricks(frame)

            closest_cx, closest_cy = self.closest_brick(brick_boxes)

            aligned = self.align_to_brick(closest_cx)
            if aligned:
                approached = self.approach(closest_cy)
                if approached:
                    break

            for box in brick_boxes:
                x1, y1, x2, y2 = box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)

            cv2.imshow("Detection Frame", frame)

            key = cv2.waitKey(20) & 0xFF

            if key == ord('q'):
                break



# Test Control Loop #
def main():
    test = Detector()

    video_path = "video_1.mp4"

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to open video file: {video_path}")
        return

    while True:
        # Get next frame
        ret, frame = cap.read()
        if not ret:
            print("Restarting video...")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        if frame is None:
            continue

        cone_boxes, cone_box_centers = test.get_cones(frame)

        for box in cone_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

        bot_boxes = test.get_bots(frame)

        for box in bot_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color=(255, 0, 0), thickness=2)

        brick_boxes = test.get_closet_bricks(frame)

        closest_cx, closest_cy = test.closest_brick(brick_boxes)

        aligned = test.align_to_brick(closest_cx)
        if aligned:
            test.approach(closest_cy)

        for box in brick_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)


        # Display Capture
        cv2.imshow("YOLO Detection on Grayscale Video", frame)

        key = cv2.waitKey(20) & 0xFF

        if key == ord('q'):
            break
                 
    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()