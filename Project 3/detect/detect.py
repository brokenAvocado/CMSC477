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
    def __init__(self):
        return
    

    def get_cones_simple(self, model, frame):
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

    def get_cones_no_merge(self, model, frame):
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
    def get_cones(self, model, frame):
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
    
    def get_bots(self, model, frame):
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


# Test Control Loop #
def main():
    test = Detector()

    video_path = "video_1.mp4"

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to open video file: {video_path}")
        return

    print('Loading models...')
    cone_model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\detect\\cone_model\\weights\\best.pt")
    bot_model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\detect\\bot_model\\weights\\best.pt")


    while True:
        # Get next frame
        ret, frame = cap.read()
        if not ret:
            print("Restarting video...")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        if frame is None:
            continue

        cone_boxes, cone_box_centers = test.get_cones(cone_model, frame)

        for box in cone_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

        # test.draw_lines_between_pairs(frame, box_centers)

        # group1, group2 = test.group_slopes(box_centers)

        # corners = test.get_corners(group1, group2)

        # test.draw_cone_corners(frame, corners)

        bot_boxes = test.get_bots(bot_model, frame)

        for box in bot_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color=(255, 0, 0), thickness=2)

        # Display Capture
        cv2.imshow("YOLO Detection on Grayscale Video", frame)

        key = cv2.waitKey(20) & 0xFF

        if key == ord('q'):
            break
                 
    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()