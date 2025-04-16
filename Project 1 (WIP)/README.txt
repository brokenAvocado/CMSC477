Purpose: Must create robot navigation program for a maze in the real world
How: combine a path finding algorithm (Djikstra or A*), motion control for
    robot, and camera to know where we are in the world

-------------------------------------------------------------------------------

Path finding portion: (1 and 2 easy, 3 is really hard)
1. Need to have correct map (for Djikstra to work) [done]
2. Find the best path from start to finish as a set of position nodes [done]
3. Interpolate the best path from the initial nodes for smooth motion (make 
    more nodes)

Motion control: (all of these should be easy)
1. For each set of positions, need a way of tracking error between expected
    and actual
2. Use velocity control (feed forward and feed backward) to follow the path
    (this is using both PI and a feed forward term)
3. Use rotation around z-axis to follow and keep April tags in frame (or else
    the pose estimation gets wonky)
4. Without delay, move from position to position

Camera pose: (1 is easy, 2 is medium, and 3 should also be easy)
1. Gather orientation and relative position from one or more April tags 
    (in real world dimensions?)
2. Based on April tag ID, find the global position (aka the robot's position 
    in the maze)
3. Correlate the real world dimensions to the path finder's dimensions (this
    might have to be done in the path finding section)

-------------------------------------------------------------------------------

Project progression:
1. Prepare pathfinding algorithm and the interpolated path
2. Work to first get accurate real world to maze world transition (move the robot
    manually around the maze and graph it on the laptop)
3. Trace the optimal path manually to see how well error is being compensated
4. Transition to robot control only, using the same functions to live graph 
    the robot's movement across the maze