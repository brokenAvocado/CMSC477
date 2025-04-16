import pupil_apriltags
import cv2
import numpy as np
import sys

class AprilTagDetector: # Given
    def __init__(self, family="tag36h11", threads=2, marker_size_m=0.16):
        K = np.array([[314, 0, 320], [0, 314, 180], [0, 0, 1]]) # Camera focal length and center pixel
        marker_size_m = 0.153 # Size of the AprilTag in meters
        self.camera_params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        self.marker_size_m = marker_size_m
        self.detector = pupil_apriltags.Detector(family, threads)

    def find_tags(self, frame_gray):
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections

    def draw_detections(self, frame, detections): # Given
        for detection in detections:
            pts = detection.corners.reshape((-1, 1, 2)).astype(np.int32)

            frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

            top_left = tuple(pts[0][0])  # First corner
            top_right = tuple(pts[1][0])  # Second corner
            bottom_right = tuple(pts[2][0])  # Third corner
            bottom_left = tuple(pts[3][0])  # Fourth corner
            cv2.line(frame, top_left, bottom_right, color=(0, 0, 255), thickness=2)
            cv2.line(frame, top_right, bottom_left, color=(0, 0, 255), thickness=2)

    def get_pose_camera_frame(self, detection):
        R_ca = detection.pose_R
        t_ca = detection.pose_t

        roll = np.arctan2(R_ca[1][0],R_ca[0][0])
        yaw = np.arctan2(-R_ca[2][0],np.sqrt(R_ca[2][1]**2+R_ca[2][2]**2))
        pitch = np.arctan2(R_ca[2][1],R_ca[2][2])
        const = 180/np.pi
        
        rotation = [const*roll, const*yaw, const*pitch]

        t_ca = t_ca.flatten()
        t_ca[2] = t_ca[2]*np.cos(pitch)
        t_ca[0] = t_ca[0]*np.cos(yaw)

        return t_ca, rotation
    
    def closest(self,detections):
        closestDist = float('inf')
        closestTag = 0

        for tag in detections:
            pos, rot = self.get_pose_camera_frame(tag)
            if np.linalg.norm([pos[0], pos[2]]) < closestDist:
                closestDist = np.linalg.norm([pos[0], pos[2]])
                closestTag = tag
        if closestTag == 0:
            return None
        
        return closestTag