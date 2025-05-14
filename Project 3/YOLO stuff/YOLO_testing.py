import cv2
import os
import random
import shutil
import time
from queue import Empty
import shutil
import re
import numpy as np
from itertools import combinations
from ultralytics import YOLO

# import robomaster
# from robomaster import robot
# from robomaster import camera

class YOLO_tester:
    def __init__(self):
        #self.robot = robot.Robot()
        return
    
    def collect_images_laptop(self):
        # Prompt user for folder name
        folder_name = input("Enter the folder name where you want to save images: ")

        # Get the current working directory
        script_dir = os.getcwd()
        save_path = os.path.join(script_dir, folder_name)

        # Create folder if it doesn't exist
        os.makedirs(save_path, exist_ok=True)

        # Start video capture (0 = default camera)
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("Error: Could not open webcam.")
            exit()

        print("Press 'm' to capture an image. Press 'q' to quit.")

        img_counter = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            cv2.imshow("Laptop Camera", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('m'):
                # Save the image
                filename = f"capture_{img_counter}.jpg"
                filepath = os.path.join(save_path, filename)
                cv2.imwrite(filepath, frame)
                print(f"Captured: {filepath}")
                img_counter += 1

            elif key == ord('q'):
                print("Exiting...")
                break

        # Release resources
        cap.release()
        cv2.destroyAllWindows()

    def collect_images_robot(self):
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")
        ep_arm = ep_robot.robotic_arm
        ep_gripper = ep_robot.gripper
        ep_chassis = ep_robot.chassis
        
        # Camera Init
        ep_camera = ep_robot.camera

        # Prompt user for folder name
        folder_name = input("Enter the folder name where you want to save images: ")

        # Get the current working directory
        script_dir = os.getcwd()
        save_path = os.path.join(script_dir, folder_name)

        # Create folder if it doesn't exist
        os.makedirs(save_path, exist_ok=True)

        # Start video capture (0 = default camera)
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

        print("Press 'm' to capture an image. Press 'q' to quit.")

        img_counter = 0

        # Movement parameters
        MOVE_DIST = 0.2  # meters
        ROTATE_ANGLE = 15  # degrees
        MOVE_SPEED = 0.7  # m/s
        ROTATE_SPEED = 45  # deg/s

        # Arm movement parameters
        ARM_STEP = 10  # degrees per press

        while True:
            # if keyboard.is_pressed("w"):
            #     ep_chassis.move(x=MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("s"):
            #     ep_chassis.move(x=-MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("a"):
            #     ep_chassis.move(x=0, y=-MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("d"):
            #     ep_chassis.move(x=0, y=MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("q"):
            #     ep_chassis.move(x=0, y=0, z=ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("e"):
            #     ep_chassis.move(x=0, y=0, z=-ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            # elif keyboard.is_pressed("up"):
            #     ep_robot.gripper.move(arm=-ARM_STEP).wait_for_completed()  # Replace with correct API
            # elif keyboard.is_pressed("down"):
            #     ep_robot.gripper.move(arm=ARM_STEP).wait_for_completed()   # Replace with correct API
            # elif keyboard.is_pressed("esc"):
            #     print("Exiting control...")
            #     break
            # time.sleep(0.01)

            try:
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

                cv2.imshow("Laptop Camera", img)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('m'):
                    # Save the image
                    filename = f"capture_{img_counter}.jpg"
                    filepath = os.path.join(save_path, filename)
                    cv2.imwrite(filepath, img)
                    print(f"Captured: {filepath}")
                    img_counter += 1

                elif key == ord('p'):
                    print("Exiting...")
                    break
            except Empty:
                time.sleep(.001)
                continue

        # Release resources
        ep_arm.unsub_position()
        print('Waiting for robomaster shutdown')
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()

    def collect_video_robot(self):
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="sta", sn="3JKCH8800100UB")
        ep_arm = ep_robot.robotic_arm
        ep_gripper = ep_robot.gripper
        ep_chassis = ep_robot.chassis

        # Camera Init
        ep_camera = ep_robot.camera

        # Prompt user for folder name
        folder_name = input("Enter the folder name where you want to save the video: ")

        # Get the current working directory
        script_dir = os.getcwd()
        save_path = os.path.join(script_dir, folder_name)

        # Create folder if it doesn't exist
        os.makedirs(save_path, exist_ok=True)

        # Start video stream
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

        print("Press 'm' to toggle video recording. Press 'q' to quit.")

        is_recording = False
        video_writer = None
        video_counter = 0
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 format

        # Movement parameters
        MOVE_DIST = 0.2  # meters
        ROTATE_ANGLE = 15  # degrees
        MOVE_SPEED = 0.7  # m/s
        ROTATE_SPEED = 45  # deg/s

        # Arm movement parameters
        ARM_STEP = 10  # degrees per press

        while True:
            if keyboard.is_pressed("w"):
                ep_chassis.move(x=MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("s"):
                ep_chassis.move(x=-MOVE_DIST, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("a"):
                ep_chassis.move(x=0, y=-MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("d"):
                ep_chassis.move(x=0, y=MOVE_DIST, z=0, xy_speed=MOVE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("q"):
                ep_chassis.move(x=0, y=0, z=ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("e"):
                ep_chassis.move(x=0, y=0, z=-ROTATE_ANGLE, z_speed=ROTATE_SPEED).wait_for_completed()
            elif keyboard.is_pressed("up"):
                ep_robot.gripper.move(arm=-ARM_STEP).wait_for_completed()  # Replace with correct API
            elif keyboard.is_pressed("down"):
                ep_robot.gripper.move(arm=ARM_STEP).wait_for_completed()   # Replace with correct API
            elif keyboard.is_pressed("esc"):
                print("Exiting control...")
                break
            time.sleep(0.01)

            try:
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                if img is None:
                    print("Failed to grab frame.")
                    break

                cv2.imshow("Robot Camera", img)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('m'):
                    if not is_recording:
                        video_filename = f"video_{video_counter}.mp4"
                        video_filepath = os.path.join(save_path, video_filename)
                        height, width = img.shape[:2]
                        video_writer = cv2.VideoWriter(video_filepath, fourcc, 20.0, (width, height))
                        is_recording = True
                        print(f"Started recording: {video_filepath}")
                    else:
                        is_recording = False
                        video_writer.release()
                        video_writer = None
                        print("Stopped recording.")
                        video_counter += 1

                elif key == ord('p'):
                    print("Exiting...")
                    break

                if is_recording and video_writer is not None:
                    video_writer.write(img)
            except Empty:
                time.sleep(.001)
                continue

        # Cleanup
        if video_writer is not None:
            video_writer.release()
        ep_camera.stop_video_stream()
        ep_arm.unsub_position()
        ep_robot.close()
        cv2.destroyAllWindows()


    def split(self):
        # Prompt user for input
        directory = input("Enter the name of the directory containing .jpg images: ").strip()
        try:
            train_percent = int(input("Enter the percentage to use for training (0–100): ").strip())
        except ValueError:
            print("Invalid percentage. Please enter an integer.")
            return

        # Validate inputs
        if not os.path.isdir(directory):
            print(f"Error: '{directory}' is not a valid directory.")
            return

        if not (0 <= train_percent <= 100):
            print("Percentage must be between 0 and 100.")
            return

        # Get list of .jpg images
        images = [f for f in os.listdir(directory) if f.lower().endswith('.jpg')]
        if not images:
            print("No .jpg images found in the directory.")
            return

        # Shuffle image list randomly
        random.shuffle(images)

        # Calculate split index
        split_idx = int((train_percent / 100) * len(images))

        # Create train and validation directories
        train_dir = directory + "_train"
        val_dir = directory + "_validation"
        os.makedirs(train_dir, exist_ok=True)
        os.makedirs(val_dir, exist_ok=True)

        # Copy images and matching .txt label files
        for i, img in enumerate(images):
            src_img = os.path.join(directory, img)
            dst_dir = train_dir if i < split_idx else val_dir
            dst_img = os.path.join(dst_dir, img)
            shutil.copy2(src_img, dst_img)

            # Look for corresponding .txt file
            label_file = os.path.splitext(img)[0] + ".txt"
            src_label = os.path.join(directory, label_file)
            dst_label = os.path.join(dst_dir, label_file)

            if os.path.exists(src_label):
                shutil.copy2(src_label, dst_label)

        print(f"\nSplit complete:")
        print(f"  → {split_idx} images (and matching labels) copied to '{train_dir}'")
        print(f"  → {len(images) - split_idx} images (and matching labels) copied to '{val_dir}'")

    def laptop_cam(self):
        print('model')
        model = None
        # model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Red_Green Testing\\train1\\weights\\best.pt")


        # Use vid instead of ep_camera to use your laptop's webcam
        vid = cv2.VideoCapture(0)

        # ep_robot = robot.Robot()
        # ep_robot.initialize(conn_type="ap")
        # ep_camera = ep_robot.camera
        # ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

        while True:
            _, frame = vid.read()
            #frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

            if frame is not None:
                start = time.time()
                if model.predictor:
                    model.predictor.args.verbose = False
                result = model.predict(source=frame, show=False)[0]


                # DIY visualization is much faster than show=True for some reason
                boxes = result.boxes
                for box in boxes:
                    xyxy = box.xyxy.cpu().numpy().flatten()
                    cv2.rectangle(frame,
                                (int(xyxy[0]), int(xyxy[1])), 
                                (int(xyxy[2]), int(xyxy[3])),
                                color=(0, 0, 255), thickness=2)
                    
                cv2.imshow('frame', frame)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break


                # print(results)


                end = time.time()
                print(1.0 / (end-start))

    def run_model_on_video(self, video_path):
        print('Loading model...')
        model = None
        model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Robot_Brick_Detection\\train13\\weights\\best.pt")

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Unable to open video file: {video_path}")
            return

        paused = False
        frame_idx = 0

        while True:
            if not paused:
                ret, frame = cap.read()
                if not ret:
                    print("Restarting video...")
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    frame_idx = 0
                    continue
                frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
                ret, frame = cap.read()

            if frame is None:
                continue

            start = time.time()
            if model.predictor:
                model.predictor.args.verbose = False
            result = model.predict(source=frame, show=False)[0]

            # Draw boxes
            boxes = result.boxes
            threshold = 1  # Euclidean distance threshold in pixels

            # Step 1: Extract all boxes and compute centers
            raw_boxes = [box.xyxy.cpu().numpy().flatten() for box in boxes]
            centers = [((b[0] + b[2]) / 2, (b[1] + b[3]) / 2) for b in raw_boxes]

            # Step 2: Track which boxes have been merged
            used = set()
            merged_boxes = []

            for i, j in combinations(range(len(raw_boxes)), 2):
                if i in used or j in used:
                    continue
                c1, c2 = centers[i], centers[j]
                dist = np.linalg.norm(np.array(c1) - np.array(c2))
                if dist < threshold:
                    # Average the coordinates of the boxes
                    merged = (np.array(raw_boxes[i]) + np.array(raw_boxes[j])) / 2
                    merged_boxes.append(merged.astype(int))
                    used.update([i, j])

            # Add unmerged boxes
            for idx, box in enumerate(raw_boxes):
                if idx not in used:
                    merged_boxes.append(box.astype(int))

            # Step 3: Draw merged boxes
            for box in merged_boxes:
                x1, y1, x2, y2 = box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

            cv2.imshow("YOLO Video Detection", frame)

            key = cv2.waitKey(20) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
            elif key == ord('<') and paused:
                frame_idx = max(0, frame_idx - 2)  # Go back one frame, accounting for current frame
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            elif key == ord('>') and paused:
                frame_idx = frame_idx + 1
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)

            end = time.time()
            print("FPS:", round(1.0 / (end - start), 2))

        cap.release()
        cv2.destroyAllWindows()

    def run_model_on_video_gray(self, video_path):
        print('Loading model...')
        model = None
        model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Robot_robot_detection\\train16\\weights\\best.pt")

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Unable to open video file: {video_path}")
            return

        paused = False
        frame_idx = 0

        while True:
            if not paused:
                ret, frame = cap.read()
                if not ret:
                    print("Restarting video...")
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    frame_idx = 0
                    continue
                frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
                ret, frame = cap.read()

            if frame is None:
                continue

            # === Convert to grayscale ===
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # YOLO expects 3 channels, so convert back to BGR (visually still gray)
            gray_bgr = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)

            start = time.time()
            if model.predictor:
                model.predictor.args.verbose = False
            result = model.predict(source=gray_bgr, show=False)[0]

            # Draw boxes
            boxes = result.boxes
            threshold = 50  # Euclidean distance threshold in pixels

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

            for box in merged_boxes:
                x1, y1, x2, y2 = box
                cv2.rectangle(gray_bgr, (x1, y1), (x2, y2), color=(0, 0, 255), thickness=2)

            cv2.imshow("YOLO Detection on Grayscale Video", gray_bgr)

            key = cv2.waitKey(20) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
            elif key == ord('<') and paused:
                frame_idx = max(0, frame_idx - 2)
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            elif key == ord('>') and paused:
                frame_idx = frame_idx + 1
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)

            end = time.time()
            print("FPS:", round(1.0 / (end - start), 2))

        cap.release()
        cv2.destroyAllWindows()

    def combine_and_rename_images(self, folder_list, dest_folder="combined_images"):
        # Create destination folder if it doesn't exist
        os.makedirs(dest_folder, exist_ok=True)

        counter = 0  # For naming captures

        for folder in folder_list:
            if not os.path.isdir(folder):
                print(f"Skipping '{folder}': Not a valid directory.")
                continue

            for filename in sorted(os.listdir(folder)):
                if filename.lower().endswith(".jpg"):
                    src_path = os.path.join(folder, filename)
                    dest_path = os.path.join(dest_folder, f"capture_{counter}.jpg")
                    
                    shutil.copy2(src_path, dest_path)
                    counter += 1

        print(f"Copied and renamed {counter} images into '{dest_folder}'.")

    def augment_images(self):
        folder = input("Enter the folder containing .jpg images to augment: ").strip()

        if not os.path.isdir(folder):
            print(f"Error: '{folder}' is not a valid folder.")
            return

        # Gather all .jpg image filenames
        image_files = [f for f in os.listdir(folder) if f.lower().endswith('.jpg')]

        if not image_files:
            print("No .jpg images found.")
            return

        # Find the highest capture index
        capture_pattern = re.compile(r"capture_(\d+)\.jpg")
        max_index = -1
        for filename in image_files:
            match = capture_pattern.match(filename)
            if match:
                max_index = max(max_index, int(match.group(1)))

        next_index = max_index + 1

        for filename in image_files:
            path = os.path.join(folder, filename)
            img = cv2.imread(path)

            if img is None:
                continue

            # Original dimensions
            h, w = img.shape[:2]

            # Decide random augmentations
            flip = random.choice([True, False])
            rotate = random.choice([-5, 0, 5])  # degrees

            aug_img = img.copy()

            if flip:
                aug_img = cv2.flip(aug_img, 1)  # Horizontal flip

            if rotate != 0:
                center = (w // 2, h // 2)
                matrix = cv2.getRotationMatrix2D(center, rotate, 1.0)
                aug_img = cv2.warpAffine(aug_img, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)

            # Save the new image
            new_filename = f"capture_{next_index}.jpg"
            new_path = os.path.join(folder, new_filename)
            cv2.imwrite(new_path, aug_img)
            print(f"Saved: {new_filename}")
            next_index += 1

        print("Augmentation complete.")

    def to_gray(self):
        # Prompt the user for a folder name
        folder = input("Enter the folder name: ").strip()

        # Check if the folder exists
        if not os.path.isdir(folder):
            print(f"Error: '{folder}' is not a valid directory.")
            return

        # Process all .jpg files in the folder
        for filename in os.listdir(folder):
            if filename.lower().endswith('.jpg'):
                file_path = os.path.join(folder, filename)

                # Read image
                img = cv2.imread(file_path)

                if img is None:
                    print(f"Warning: Failed to read {filename}")
                    continue

                # Convert to grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Save the grayscale image (overwrite original or add _gray)
                gray_filename = os.path.splitext(filename)[0] + '.jpg'
                gray_path = os.path.join(folder, gray_filename)
                cv2.imwrite(gray_path, gray)

                print(f"Converted: {filename} → {gray_filename}")

    def brick_detect_test(self, video_path):
        print('Loading model...')
        model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Robot_Brick_Detection\\train13\\weights\\best.pt")

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Unable to open video file: {video_path}")
            return

        class_colors = {
            "small": (144, 238, 144),   # Light green
            "medium": (0, 100, 0),      # Dark green
            "large": (139, 0, 0)        # Dark blue
        }

        while True:
            ret, frame = cap.read()
            if not ret:
                print("End of video or cannot read frame.")
                break

            if model.predictor:
                model.predictor.args.verbose = False

            result = model.predict(source=frame, show=False)[0]
            boxes = result.boxes

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten().astype(int)
                cls_id = int(box.cls.item())
                class_name = model.names.get(cls_id, "unknown")
                color = class_colors.get(class_name, (0, 0, 255))  # Default to red

                cv2.rectangle(frame, (x1, y1), (x2, y2), color=color, thickness=2)
                cv2.putText(frame, class_name, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            closest = self.closest_brick(boxes, frame.shape) 
            if closest is not None:
                cx = (closest[0] + closest[2]) // 2
                cy = (closest[1] + closest[3]) // 2
                cv2.circle(frame, (cx, cy), 10, (255, 0, 255), 2)  # Purple circle
                self.align_to_brick(cx, frame.shape[1])

            cv2.imshow("YOLO Video Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def brick_detect_bot(self):
        print('Loading model...')
        model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Robot_Brick_Detection\\train13\\weights\\best.pt")

        # Initialize robot and camera if not already done
        if not hasattr(self, 'robot'):
            self.robot = robot.Robot()
            self.robot.initialize(conn_type="sta")

        ep_camera = self.robot.camera
        ep_camera.start_video_stream(display=False)

        class_colors = {
            "small": (144, 238, 144),   # Light green
            "medium": (0, 100, 0),      # Dark green
            "large": (139, 0, 0)        # Dark blue
        }

        print("Press 'q' to quit.")
        while True:
            # Get frame from RoboMaster camera
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if frame is None:
                print("Failed to grab frame.")
                continue

            if model.predictor:
                model.predictor.args.verbose = False

            result = model.predict(source=frame, show=False)[0]
            boxes = result.boxes

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten().astype(int)
                cls_id = int(box.cls.item())
                class_name = model.names.get(cls_id, "unknown")
                color = class_colors.get(class_name, (0, 0, 255))  # Default to red

                cv2.rectangle(frame, (x1, y1), (x2, y2), color=color, thickness=2)
                cv2.putText(frame, class_name, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Get and mark the closest brick
            closest = self.closest_brick(boxes, frame.shape)
            if closest is not None:
                cx = (closest[0] + closest[2]) // 2
                cy = (closest[1] + closest[3]) // 2
                cv2.circle(frame, (cx, cy), 10, (255, 0, 255), 2)  # Purple circle

                self.align_to_brick(cx, frame.shape[1])
                self.approach(cy, frame.shape[0])

            # Show annotated frame
            cv2.imshow("YOLO Live Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()

    def closest_brick(self, boxes, frame_shape):
        if not boxes:
            return None

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

        return closest

    def align_to_brick(self, cx, frame_width, threshold=40):
        center_x = frame_width // 2
        offset = cx - center_x

        if abs(offset) < threshold:
            print("Brick is centered.")
            return  # No turn needed

        # Initialize robot connection once
        chassis = self.robot.chassis

        if offset < 0:
            print("Brick is left — turning left")
            chassis.rotate(angle=-5)  # Turn left 5 degrees
        else:
            print("Brick is right — turning right")
            chassis.rotate(angle=5)   # Turn right 5 degrees

    def approach(self, cy, frame_height, tolerance=30, speed=0.2):
        # Initialize robot if needed
        chassis = self.robot.chassis
        bottom_threshold = frame_height - tolerance

        if cy < bottom_threshold:
            print(f"Brick is not close enough (cy = {cy}) — driving forward.")
            chassis.move(x=0.1, y=0, z=0, xy_speed=speed).wait_for_completed()
        else:
            print(f"Brick is within approach threshold (cy = {cy}) — stopping.")



# Example usage:
# augment_images()

def main():
    test = YOLO_tester()
    # test.collect_images_robot()
    # test.collect_video_robot()
    test.split()
    #test.laptop_cam()
    # test.combine_and_rename_images(["robot_corridor_images0", "robot_corridor_images1", "robot_corridor_images2", "robot_corridor_images3", "robot_corridor_images4"])
    #test.brick_detect_test("video_0.mp4")
    # test.run_model_on_video_gray("video_0.mp4")
    # test.augment_images()
    #test.to_gray()

if __name__ == "__main__":
    main()


