import pupil_apriltags
import cv2
import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class AprilTagDetector: # Given
    def __init__(self, family="tag36h11", threads=2, marker_size_m=0.16):
        K = np.array([[314, 0, 320], [0, 314, 180], [0, 0, 1]]) # Camera focal length and center pixel
        marker_size_m = 0.153 # Size of the AprilTag in meters
        self.camera_params = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        self.marker_size_m = marker_size_m
        self.detector = pupil_apriltags.Detector(family, threads)

        self.boxWidth = 0.266
        self.boxRadius = self.boxWidth*2**0.5
        self.seenTags = []
        self.obstacles = {}

    def find_tags(self, frame_gray):
        '''
        Finds any and all tags in camera frame
        '''
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections
    
    def init_tags(self, detections):
        '''
        At the beginning, robot's orientation is known and the tags it sees 
        will be tracked at their global orientation 
        '''
    
    def refine_tags(self, detections, robot_global_pose, tol=0.1):
        '''
        Using positional data, will detect if the tag is a new tag and will check if it's
        a tag from a box the robot has seen before

        Tags are always put into a list of "seen" tags and tags that meet the unique criteria
        will average the box's global position
        '''
        for tag in detections:
            if not tag.tag_id in self.seenTags: # For tags that are seen for the first time
                self.seenTags.append(tag.tag_id)

                x,y = self.get_global_pos(tag, robot_global_pose)
                pose = np.array([x, y])

                if self.obstacles: # Check if not empty
                    for ob_key in self.obstacles:
                        pose_stored = self.obstacles[ob_key]
                        if np.norm(pose-pose_stored) <= tol:
                            self.obstacles[ob_key] = (pose + pose_stored)/2
                else:
                    self.obstacles[tag.tag_id] = pose

    def get_relative_pos(self, detection):
        '''
        Returns relative positional and rotational data from a detection

        Position is at the predicted center of the box (using the orientation 
            vector and the box width)
        '''
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
        t_ca[2] = t_ca[2]+self.boxWidth/2*np.cos(yaw)
        t_ca[0] = t_ca[0]+self.boxWidth/2*np.sin(yaw)

        return np.array(t_ca), np.array(rotation)
    
    def get_global_pos(self, detection, robot_global_pose):
        '''
        Returns the AprilTag box's global position

        Uses robot global position + orientation, then tacks on the
        obstacle's position + orientation to get the global information
        '''

        pos, rot = self.get_relative_pos(detection)
        global_pos = robot_global_pose[0]
        global_rot = robot_global_pose[1]

        return pos+global_pos, global_rot
        
    
    def movingAverage(self, data, window):
        '''
        Calculates the average of a frame of data
        '''
        sum = 0
        for ind, points in enumerate(data):
            sum += points
            if ind >= window:
                return sum/window

    def closest(self, detections):
        closestDist = float('inf')
        closestTag = 0

        for tag in detections:
            pos, rot = self.get_relative_pos(tag)
            if np.linalg.norm([pos[0], pos[2]]) < closestDist:
                closestDist = np.linalg.norm([pos[0], pos[2]])
                closestTag = tag
        if closestTag == 0:
            return None
        
        return closestTag
    
    '''
    Visualization Scripts
    '''
    
    def initGraph(self):
        _, ax = plt.subplots()
        # graph = ax.plot([],[],'go')[0]
        graph = patches.Circle((0, 0), radius=self.boxWidth)
        ax.add_patch(graph)
        # graph2 = ax.plot([],[], markersize=15, color='red')[0]
        ax.set(xlim=[-2, 2],ylim=[0, 2])
        
        return graph
    
    def draw_detections(self, frame, detections): # Given
        '''
        (Visualization) Draws AprilTags in the camera window with a red outline
        '''
        for detection in detections:
            pts = detection.corners.reshape((-1, 1, 2)).astype(np.int32)

            frame = cv2.polylines(frame, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

            top_left = tuple(pts[0][0])  # First corner
            top_right = tuple(pts[1][0])  # Second corner
            bottom_right = tuple(pts[2][0])  # Third corner
            bottom_left = tuple(pts[3][0])  # Fourth corner
            cv2.line(frame, top_left, bottom_right, color=(0, 0, 255), thickness=2)
            cv2.line(frame, top_right, bottom_left, color=(0, 0, 255), thickness=2)
    
    def plot_detections(self, detections, graph):
        '''
        (Visualization)
        Need to run initGraph for this to work
        Plots the aruco tags as cirlces on a graph
        '''
        plot_x = []
        plot_y = []
        amp = 0.2

        # closeTag = self.closest(detections)
        
        # Do the following for all tags
        for detection in detections:
            pos, rot = self.get_relative_pos(detection)
            plot_x.append(pos[0])
            plot_y.append(pos[2])

        # Plot vector for closest tag
        # if not(closeTag is None):
        #     quiver.set_xdata([pos[0], pos[0]-amp*np.sin(rot[1])])
        #     quiver.set_ydata([pos[2], pos[2]-amp*np.cos(rot[1])])

        # set the x and y data
        graph.center = plot_x, plot_y

        # for i, label in enumerate(labels):
        #     plt.text(plot_x[i], plot_y[i], label, ha='center', va='bottom')

        plt.draw()
        plt.pause(0.00001)