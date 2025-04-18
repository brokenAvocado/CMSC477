import cv2
import os
import random
import shutil
import time
import robomaster
from robomaster import robot
from robomaster import camera
import keyboard
from queue import Empty

class YOLO_tester:
    def __init__(self):
        
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
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
        model = YOLO("C:\\Users\\Trevor\\Documents\\Python Scripts\\CMSC477\\Project 3\\YOLO stuff\\Red_Green Testing\\train1\\weights\\best.pt")


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

def main():
    test = YOLO_tester()
    test.collect_video_robot()
    #test.split()
    #test.laptop_cam()

if __name__ == "__main__":
    main()


