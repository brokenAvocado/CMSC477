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
        self.obstacles = {
            32: [],
            46: [],
            7: [8, 9, 10],
            8: [7, 9, 10],
            9: [7, 8, 10],
            10: [7, 8, 9],
            11: [15, 19, 23],
            15: [11, 19, 23],
            19: [11, 15, 23],
            23: [11, 15, 19],
            12: [16, 20, 24],
            16: [12, 20, 24],
            20: [12, 16, 24],
            24: [12, 16, 20],
            13: [17, 21, 25],
            17: [13, 21, 25],
            21: [13, 17, 25],
            25: [13, 17, 21],
            14: [18, 22, 26],
            18: [14, 22, 26],
            22: [14, 18, 26],
            26: [14, 18, 22],
            33: [34, 37, 38],
            34: [33, 37, 38],
            37: [33, 34, 38],
            38: [33, 34, 37],
            40: [41, 44, 45],
            41: [40, 44, 45],
            44: [40, 41, 45],
            45: [40, 41, 44],
            27: [28, 30, 31],
            28: [27, 30, 31],
            30: [27, 28, 31],
            31: [27, 28, 30]
        }
        self.obstaclesX = {}
        self.obstaclesY = {}
        self.obstaclesPointers = {}
        self.obstaclePos = {}

    def find_tags(self, frame_gray):
        '''
        Finds any and all tags in camera frame
        '''
        detections = self.detector.detect(frame_gray, estimate_tag_pose=True,
            camera_params=self.camera_params, tag_size=self.marker_size_m)
        return detections
    
    def troubleshoot(self):
        print(f"Obstacle Pointers {self.obstaclesPointers} Obstacle Pos {self.obstaclePos}")
    
    def refine_tags(self, detection, robot_global_pose, window):
        '''
        Using positional data, will detect if the tag is a new tag and make an empty
        array inside a dictionary.

        For existing tags, fill the array corresponding to that tag in the dictionary
        unitl a window is reached

        Reset the pointer based on the window
        '''
        
        id = detection.tag_id
        boxPosition = self.get_box_global(detection, robot_global_pose)

        # If this is a new pointer, make the pointer start from 0 and a new array
        if id not in self.obstaclesPointers:
            self.obstaclesPointers[id] = 0
            self.obstaclesX[id] = [boxPosition[0]]
            self.obstaclesY[id] = [boxPosition[1]]
        else:

            if len(self.obstaclesX[id]) < window:
                self.obstaclesX[id].append(boxPosition[0])
            else:
                self.obstaclesX[id][self.obstaclesPointers[id]] = boxPosition[0]

            if len(self.obstaclesY[id]) < window:
                self.obstaclesY[id].append(boxPosition[1])
            else:
                self.obstaclesY[id][self.obstaclesPointers[id]] = boxPosition[1]

            self.obstaclesPointers[id] += 1

            if self.obstaclesPointers[id] >= window:
                self.obstaclesPointers[id] = 0

    def movingAvg_tags(self, detections, robot_global_pose, window = 5):
        '''
        If the array is a valid size, the moving average for the obstacle is calculated

        Generalizes from companion tags and compiles it into one values for each box (in global coords)
        '''
        for detection in detections:
            id = detection.tag_id
            changeAvg = False

            self.refine_tags(detection, robot_global_pose, window)
            
            # Use the companion dictionary to find if the AprilTag is already logged in obstacles
            companions = self.obstacles[id]

            if id not in self.obstaclePos:
                for companion_id in companions:
                    # If one of the companion ID is found, then break out of the for loop
                    if companion_id in self.obstaclePos:
                        break

                    # If none of the companion ID matches, then turn this boolean on
                    changeAvg = True
            else:
                changeAvg = True
            
            if changeAvg and self.obstaclesX[id] and len(self.obstaclesX[id]) == window:
                # Update the existing dictionary entry
                allId = [id]
                allId.extend(companions)
                self.obstaclePos[id] = self.average(allId)
                    
    def average(self, allId):
        '''
        Performs a sample average based on all the tags related to the box
        '''
        totalSumX = 0
        totalSumY = 0
        length = 0
        n = 0 # Number of valid dictionaries

        for tags in allId:
            if tags in self.obstaclesX and tags in self.obstaclesY:
                totalSumX += np.sum(self.obstaclesX[tags])
                totalSumY += np.sum(self.obstaclesY[tags])
                length = len(self.obstaclesX[tags])
                n += 1

        return [totalSumX/(n*length), totalSumY/(n*length)]
            
    def get_box_relative(self, detection):
        '''
        Returns relative positional and rotational data from a detection

        Position is at the predicted center of the box (using the orientation 
            vector and the box width)

        The axis of the robot is relatively: x = side to side (right is positive),
        y = front and back (forwards is positive)
        '''
        R_ca = detection.pose_R
        t_ca = detection.pose_t

        roll = np.arctan2(R_ca[1][0],R_ca[0][0])
        yaw = np.arctan2(-R_ca[2][0],np.sqrt(R_ca[2][1]**2+R_ca[2][2]**2))
        pitch = np.arctan2(R_ca[2][1],R_ca[2][2])
        const = 1#180/np.pi
        
        # rotation = [const*roll, const*yaw, const*pitch]

        t_ca = t_ca.flatten()
        # t_ca[2] = t_ca[2]*np.cos(pitch)
        # t_ca[0] = t_ca[0]*np.cos(yaw)
        t_ca[2] = t_ca[2]+self.boxWidth/2*np.cos(yaw)
        t_ca[0] = t_ca[0]+self.boxWidth/2*np.sin(yaw)

        return [t_ca[0], t_ca[2], yaw]
    
    def get_box_global(self, detection, robot_global_pose):
        '''
        Returns the AprilTag box's global position

        Uses robot global position + orientation, then tacks on the
        obstacle's position + orientation to get the global information
        '''
        pose = self.get_box_relative(detection)
        box_posX = pose[0]
        box_posY = pose[1]
        box_rot = 180/np.pi*np.arctan2(box_posX, box_posY)
        global_posX = robot_global_pose[0]
        global_posY = robot_global_pose[1]
        global_rot = robot_global_pose[2]

        box_g_rot = -global_rot+box_rot
        box_dist = (box_posX**2+box_posY**2)**0.5
        final_posX = global_posX + box_dist*np.sin(np.pi/180*box_g_rot)
        final_posY = global_posY + box_dist*np.cos(np.pi/180*box_g_rot)
        # print(f"Box Position {detection.tag_id}: {box_posX}, {box_posY}, {box_g_rot}, {box_rot} ")
        # print(f"Robot Position: {global_posX}, {global_posY}, {global_rot}")

        return [final_posX, final_posY]
    
    # def get_robot_global(self, detection):
    #     '''
    #     Using the AprilTag box global position, finds the robot using the box position
    #     Uses triangulation from multiple boxes to find the position of the robot
    #     '''

    #     for tags in detection:

    def closestPosition(self, detections):
        closestDist = float('inf')
        closestTag = 0

        for tag in detections:
            pose = self.get_box_relative(tag)
            pos = pose[0:2]
            if np.linalg.norm([pos[0], pos[1]]) < closestDist:
                closestDist = np.linalg.norm([pos[0], pos[1]])
                closestTag = tag
        if closestTag == 0:
            return None
        
        return self.get_box_relative(closestTag)
    
    '''
    Visualization Scripts
    '''
    
    def initGraph(self):
        _, ax = plt.subplots()
        graph = ax.plot([],[],'go')[0]
        # graph = patches.Circle((0, 0), radius=self.boxWidth)
        # ax.add_patch(graph)
        graph2 = ax.plot([],[], 'o', markersize=15, color='red')[0]
        graph3 = ax.plot([],[], color='red')[0]
        ax.set(xlim=[-1, 5] ,ylim=[-1, 10])
        
        return graph, graph2, graph3
    
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
    
    def plot_detections(self, roboPose, graph1, graph2, graph3):
        '''
        (Visualization)
        Need to run initGraph for this to work
        Plots the aruco tags as cirlces on a graph
        '''
        plot_x = []
        plot_y = []

        vector_x = []
        vector_y = []
        amp = 0.2

        # closeTag = self.closest(detections)
        
        # Do the following for all tags
        for pos in list(self.obstaclePos.values()):
            plot_x.append(pos[0])
            plot_y.append(pos[1])

        # Plot the robot's orientation
        vector_x = [roboPose[0], roboPose[0]+np.cos(np.pi/180*(roboPose[2]+90))]
        vector_y = [roboPose[1], roboPose[1]+np.sin(np.pi/180*(roboPose[2]+90))]

        # set the x and y data
        # graph.center = plot_x, plot_y
        graph1.set_xdata(plot_x)
        graph1.set_ydata(plot_y)

        graph2.set_xdata(roboPose[0])
        graph2.set_ydata(roboPose[1])

        graph3.set_xdata(vector_x)
        graph3.set_ydata(vector_y)

        # for i, label in enumerate(labels):
        #     plt.text(plot_x[i], plot_y[i], label, ha='center', va='bottom')

        plt.draw()
        plt.pause(0.00001)