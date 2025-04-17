Objective: robot needs to swap the position of two objects between two spots
    while accounting for moving objects and moving spots
How to accomplish:
    1. State Machine to know when to operate the arm or move the robot
    2. Object detection to create bounding boxes for objects and targets
    3. Perception to measure distances based on bounding box sizes
    4. Motion to move between targets and precise enough to pick targets up
    5. Arm-Motion to precisely grasp objects and move around with them

--------------------------------------------------------------------------------------------
SYSTEM BREAKDOWN

State Machine - EASY
    - Underlying operational logic to get a robust state machine working
    - Has to overcome both a moved objects not being in the same position 
    and a target not being in the same position

Object Detection - DIFFICULT
    - Accurate enough through training and usage to be able to measure distances
    - Robust enough to be able to identify objects despite being far away

Perception - DIFFICULT
    - Translate the bounding box sizes to real world distances
    - Precise perception when close to objects to be able to position for grasping
    - Have confident enough distances to move the robot from afar to up close
    - Memorize the location of objects (or have the robot just look for the object)

Motion - MEDIUM
    - Use a control loop to move between desired points precisely
    - Be able to adjust motion from fast to slow when up close
    - Precise and consistent movement to get close to a block to pick it up with
    arm (every time)

Arm Motion - MEDIUM
    - Use arm control loop to know how deep to go to pick up a block
    - Grasp strength and angle to grasp that will ensure the tower is picked up
    - Height to raise the arm after grasped so movement isn't impaired

--------------------------------------------------------------------------------------
MAJOR CHALLENGES

