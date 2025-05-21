%% Project 3: Data Plotting

clc;
clearvars -except data_10;
close all;

robot_pos = data_10(:,1:2);
target_pos = data_10(:,3:4);
box_pos = data_10(:,5:6);
transitions = [19, 71, 133, 201, 246, 309, 361, 423, 460, 510, 574, 620, ...
    694, 753, 798, 863, 915, 984, 1023, 1069, 1127, 1168, 1233, 1296, 1340, ...
    1404, 1452, 1518, 1559, 1607, 1673, 1719];

events = {"   Evade April Tag",{"Avoided","April Tag"}, "Closet 1", {"Transition","Point"}, {"Hallway","Entry"},...
    {"Hallway","Exit"}, {"Transition","Point"}, "Closet 2"};

robotDelta = robot_pos-target_pos;
robotDistance = sqrt(robotDelta(:,1).^2 + robotDelta(:,2).^2);

targetDelta = target_pos;
targetDist = sqrt(targetDelta(:,1).^2 + targetDelta(:,2).^2);

boxDelta = robot_pos-box_pos;
boxDist = sqrt(boxDelta(:,1).^2 + boxDelta(:,2).^2);

figure
hold on
plot(robot_pos(:,1),"r")
plot(target_pos(:,1),"r--")
hold off
title("Robot X Global Position")
xlabel("Sample #")
ylabel("Arena Position (m)")
legend(["Actual X", "Target X"])

figure
hold on
plot(robot_pos(:,2),"b")
plot(target_pos(:,2),"b--")
hold off
title("Robot Y Global Position")
xlabel("Sample #")
ylabel("Arena Position (m)")
legend(["Actual Y", "Target Y"])

figure
hold on
plot(robotDistance, "m")
% plot(0.15*ones(size(boxDist)), "r:")
hold off
title("Robot Distance to Target")
xlabel("Sample #")
ylabel("Distance (m)")

figure
hold on
plot(robotDistance(1:423), "m")
scatter(transitions(1:8), robotDistance(transitions(1:8)))
text(transitions(1:8), robotDistance(transitions(1:8))-0.1, events)
% plot(0.15*ones(size(boxDist)), "r:")
hold off
title("Robot Distance to Target (First Cycle)")
xlabel("Sample #")
ylabel("Distance (m)")
legend("Actual Distance")
ylim([-1 3.5])

figure
hold on
plot(boxDist)
plot(0.85*ones(size(boxDist)), "r:")
hold off
title("Robot Distance to Closest Box")
xlabel("Sample #")
ylabel("Distance (m)")
legend(["Distance", "Box Avoidance Tolerance"])

figure
hold on
plot(boxDist(1:694))
plot(0.85*ones(size(boxDist(1:694))), "r:")
hold off
title("Robot Distance to Closest Box (First Cycle)")
xlabel("Sample #")
ylabel("Distance (m)")
legend(["Distance", "Box Avoidance Tolerance"])