Motion Planning:
Main Objective: move the robot from point A to point B without bumping into objects and other robots
Sub-Objectives:
    - Wihtout a map, have the robot know where it starts and where it 
        needs to go from that start position
        [homing procedure]

    - Somewhere along the way, update and correct its global positioning 
        with landmarks (potentially the first set of aruco tags it detected)
        [some function to recalibrate while on the move]

    - Set up waypoints for the robot to move to in the global map
        [dictionary of waypoints (e.g. hallway, closet 1, closet 2, room 1, room 2)]
    
    - Update said waypoints by keeping track of objects and obstacles on the field
        [dictionary of sub-waypoints based on obstacle avoidance, separate from main waypoints]

    - Memorize waypoints to avoid recalculating them
        [separate functions for checking obstacles and one for checking and rewriting obstacles]

    - Keep track of object positions
        [finding the center of boxes, filling a dictionary]

To-do (functions):
    - Box averaging script (takes in new detections, relates them to old)
    - Recalibrate global position of the robot based on the robot's scan of boxes
    - Recalibrate the new box positions
    - Visual servo-ing based on where the boxes are 