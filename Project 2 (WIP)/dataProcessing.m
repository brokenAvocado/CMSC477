clc; clear; close all;

data1 = importdata("gooddata_run1_edit.txt");
data2 = importdata("gooddata_run2.txt");

%% Data 1 Plots
% % Combined
% figure
% yyaxis left
% plot(data1(:,2))
% ylim([0 0.9])
% ylabel("Distance (m)")
% title("Robot Telemetry Graph (Uninterrupted)")
% 
% yyaxis right
% plot(-data1(:,1))
% ylim([-0.5 400])
% ylabel("Pixels")
% xlabel("Sample Number")
% xline(1202, '-', {"Green Block","Found"})
% xline(3485, '-', {"Orange Pad","Found"})
% xline(4013, '-', {"Red Block","Found"})
% xline(4861, '-', {"Purple Pad","Found"})

% Pixel plot
% figure
% plot(-data1(:,1))
% ylim([0 40])
% xline(1202, '-', {"Green Block","Found"})
% xline(3485, '-', {"Orange Pad","Found"})
% xline(4013, '-', {"Red Block","Found"})
% xline(4861, '-', {"Purple Pad","Found"})
% title("Pixel Plot Over Sample Entry (Uninterrupted)")
% xlabel("Sample Number")
% ylabel("Pixels (px)")
% 
% % Distance plot
% figure
% plot(data1(:,2))
% xline(1202, '-', {"Green Block","Found"})
% xline(3485, '-', {"Orange Pad","Found"})
% xline(4013, '-', {"Red Block","Found"})
% xline(4861, '-', {"Purple Pad","Found"})
% title("Distance Plot Over Sample Entry (Uninterrupted)")
% xlabel("Sample Number")
% ylabel("Distance (m)")

%% Data 2 Plots
% Combined
figure
yyaxis left
plot(data2(:,2))
ylim([-0.3 0.9])
ylabel("Distance (m)")
title("Robot Telemetry Graph (Interrupted)")

yyaxis right
plot(data2(:,1))
ylim([-100 400])
xline(0, '--', {"Red Block","Found"})
xline(1270, '--', {"Green Block","Found"})
xline(2762, '--', {"Orange Pad","Found"})
xline(3543, '--', {"Red Block","Found"})
xline(5401, '--', {"Purple Pad","Found"})
ylabel("Pixels")
xlabel("Sample Number")

% Pixel plot
figure
plot(data2(:,1))
ylim([-80 50])
xline(0, '--', {"Red Block","Found"})
xline(1270, '--', {"Green Block","Found"})
xline(2762, '--', {"Orange Pad","Found"})
xline(3543, '--', {"Red Block","Found"})
xline(5401, '--', {"Purple Pad","Found"})
title("Pixel Plot Over Sample Entry (Interrupted)")
xlabel("Sample Number")
ylabel("Pixels (px)")

% Distance plot
figure
plot(data2(:,2))
xline(0, '--', {"Red Block","Found"})
xline(1270, '--', {"Green Block","Found"})
xline(2762, '--', {"Orange Pad","Found"})
xline(3543, '--', {"Red Block","Found"})
xline(5401, '--', {"Purple Pad","Found"})
title("Distance Plot Over Sample Entry (Interrupted)")
xlabel("Sample Number")
ylabel("Distance (m)")
