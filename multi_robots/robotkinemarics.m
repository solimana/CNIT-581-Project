
%% Path Following for a Differential Drive Robot
%
%https://www.mathworks.com/help/robotics/ug/path-following-for-differential-drive-robot.html
%
    %%
    clear
    clc 
    close all
    
path = [5.00    -2.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   -10.00];
    path = [linspace(path(1,1),path(end,1),10);linspace(path(1,2),path(end,2),10)]';
  robotInitialLocation = path(1,:);
robotGoal = path(end,:);  
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 0.122, "VehicleInputs", "VehicleSpeedHeadingRate");
figure
plot(path(:,1), path(:,2),'k--d')
xlim([5 13])
ylim([-10 -2])
%%
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
plot(4,6,'*','LineWidth',5)
hold on
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
%      https://www.mathworks.com/help/robotics/ug/control-a-differential-drive-robot-in-simulink-and-gazebo.html
%     [v, omega]
    % Get the robot's velocity using controller inputs
    phi_l = (1/(0.065))*(v-omega*0.122/2);
    phi_r = (1/(0.065))*(v+omega*0.122/2);
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")

    hold all
        plot(4,6,'*','LineWidth',5)

    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([5 13])
ylim([-10 -2])
    
    
    waitfor(vizRate);
end