%% load an display
clear
clc

addpath('Assem199\urdf\') %adding the correct folder
robot = createRigidBodyTree; %adding the robot from the createrigidbodytree function
%%
axes = show(robot);  %shows the robotic arm
axes.CameraPositionMode = 'auto';  %automated visual position

wayPoints = [0 -0.75 0.5;0 -0.75 0.7;1 0 0.5;0 1.4 0.5;0 1.4 0;]; %creates a route 

hold on
exampleHelperPlotWaypoints(wayPoints); %plots the waypoints,from the function: exampleHelperPlotWaypoints
trajectory1=cscvn(wayPoints'); %creates a smooth trajectory based on the waypoints
fnplt(trajectory1,'r',2); %plots the trajectory

%%

numTotalPoints = size(wayPoints,1)*20; %multiples the amount of waypoints by ten
waypointTime = 4;
trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime); %creates a trajectory
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.5 0.5 0.5 1 1 1];
initialguess = robot.homeConfiguration;
B =  trajectory.';  %flips the dimmensions a correctly dimmentioned matracie

for idx = 1:numTotalPoints
    
    tform = trvec2tform(B(idx,:));
    configSoln(idx, :) = ik('gripper_center', tform, weights, initialguess);
   
    initialguess = configSoln(idx,:);
    
end

%% Visualize robot configurations
title('Robot waypoint tracking visualization')

for idx = 1:numTotalPoints
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.2)
end
hold off