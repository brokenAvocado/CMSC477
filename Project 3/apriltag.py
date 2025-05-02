import pupil_apriltags
import cv2
import numpy as np
import sys
import matplotlib.pyplot as plt

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

    def plot_detections(self, detections, graph, quiver):
        '''
        Need to run initGraph for this to work
        Plots the aruco tags as points on a graph
        '''
        plot_x = []
        plot_y = []
        amp = 0.2

        closeTag = self.closest(detections)
        
        # Do the following for all tags
        for detection in detections:
            pos, rot = self.get_pose_camera_frame(detection)
            plot_x.append(pos[0])
            plot_y.append(pos[2])

        # Plot vector for closest tag
        if not(closeTag is None):
            quiver.set_xdata([pos[0], pos[0]-amp*np.sin(rot[1])])
            quiver.set_ydata([pos[2], pos[2]-amp*np.cos(rot[1])])

        # set the x and y data
        graph.set_xdata(plot_x)
        graph.set_ydata(plot_y)

        # for i, label in enumerate(labels):
        #     plt.text(plot_x[i], plot_y[i], label, ha='center', va='bottom')

        plt.draw()
        plt.pause(0.00001)


    def get_pose_camera_frame(self, detection):
        R_ca = detection.pose_R
        t_ca = detection.pose_t

        roll = np.arctan2(R_ca[1][0],R_ca[0][0])
        yaw = np.arctan2(-R_ca[2][0],np.sqrt(R_ca[2][1]**2+R_ca[2][2]**2))
        pitch = np.arctan2(R_ca[2][1],R_ca[2][2])
        const = 1#180/np.pi
        
        rotation = [const*roll, const*yaw, const*pitch]

        t_ca = t_ca.flatten()
        # t_ca[2] = t_ca[2]*np.cos(pitch)
        # t_ca[0] = t_ca[0]*np.cos(yaw)
        t_ca[2] = t_ca[2]
        t_ca[0] = t_ca[0]

        return t_ca, rotation
    
    def movingAverage(self, data, window):
        sum = 0
        for ind, points in enumerate(data):
            sum += points
            if ind >= window:
                return sum/window

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
    
    def initGraph(self):
        _, ax = plt.subplots()
        graph = ax.plot([],[],'go')[0]
        graph2 = ax.plot([],[], markersize=15, color='red')[0]
        ax.set(xlim=[-2, 2],ylim=[0, 2])
        
        return graph, graph2