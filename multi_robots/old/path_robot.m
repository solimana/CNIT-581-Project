%%
rosshutdown
    clear
    clc 

    close all
    
rosinit('192.168.1.7')


%%
% robposB

robotposeB = rossubscriber("/pos_robB","DataFormat","struct");
       
    [msg2] = receive(robotposeB,10);
   robposB = double(msg2.Data);
   
   robotposeB_or = rossubscriber("/qualisys/robotB/pose","DataFormat","struct");
       
    [msg2] = receive(robotposeB_or,10);
   robotposeB_orv = double([msg2.Pose.Orientation.X msg2.Pose.Orientation.Y msg2.Pose.Orientation.Z msg2.Pose.Orientation.W]);
   robotposeB_orv = rad2deg(quat2eul(robotposeB_orv,'XYZ'))
   
initialOrientation = deg2rad(robotposeB_orv(2));


st1pose = rossubscriber("/pos_st1","DataFormat","struct");
   [msg2] = receive(st1pose,10);
   st1 = double(msg2.Data); 
   
path = [linspace(robposB(3),st1(3),10);linspace(robposB(1),st1(1),10)]';
% path = float(path)
    
%     robposB
pause(1);
% path =[0.545157313346863,-0.0526841878890991;0.667452891667684,-0.113302025530073;0.789748469988505,-0.173919863171048;0.912044048309326,-0.234537700812022;1.03433962663015,-0.295155538452996;1.15663520495097,-0.355773376093970;1.27893078327179,-0.416391213734945;1.40122636159261,-0.477009051375919;1.52352193991343,-0.537626889016893;1.64581751823425,-0.598244726657867];

 robotInitialLocation = path(1,:);
robotGoal = path(end,:);  
% initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 0.122, "VehicleInputs", "VehicleSpeedHeadingRate","WheelSpeedRange",[-12 12]);
figure
plot(path(:,1), path(:,2),'k--d')
xlim([-3 3])
ylim([-3 3])

robotBpub = rospublisher("/motorsB","std_msgs/Int32MultiArray","DataFormat","struct");
robotBmsg = rosmessage(robotBpub);


%%
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.1;
goalRadius = 0.3;
distanceToGoal = norm(robotInitialLocation - robotGoal);
% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
% plot(4,6,'*','LineWidth',5)
hold on
while( distanceToGoal > goalRadius )
%     robotCurrentPose = [robposB(1);robposB(3);atan2(robposB(3),robposB(1))]
    % Compute the controller outputs, i.e., the inputs to the robot
%     robotposeB = rossubscriber("/pos_robB",@savepos_robposB,"DataFormat","struct");
%     global robposB
distanceToGoal 
%     robotposeB = rossubscriber("/pos_robB","DataFormat","struct");
%     st1pose = rossubscriber("/pos_st1","DataFormat","struct");


   [msg2] = receive(robotposeB,10);
   robposB = double(msg2.Data);
   [msg2] = receive(st1pose,10);
   st1 = double(msg2.Data);
   
          
    [msg2] = receive(robotposeB_or,10);
   robotposeB_orv = double([msg2.Pose.Orientation.X msg2.Pose.Orientation.Y msg2.Pose.Orientation.Z msg2.Pose.Orientation.W]);
   robotposeB_orv = rad2deg(quat2eul(robotposeB_orv,'XYZ'));
    corientation = deg2rad(robotposeB_orv(2));
%     corientation = 0.6;
    [v, omega] = controller(robotCurrentPose);
%      https://www.mathworks.com/help/robotics/ug/control-a-differential-drive-robot-in-simulink-and-gazebo.html
%     [v, omega]
    % Get the robot's velocity using controller inputs
    phi_l = (1/(0.065))*(v+omega*0.122/2);
    phi_r = (1/(0.065))*(v-omega*0.122/2);
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
%     robotCurrentPose = robotCurrentPose + vel*sampleTime; 
        robotCurrentPose = [robposB(3);robposB(1);corientation];

%     robotCurrentPose
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")

    hold all
%         plot(4,6,'*','LineWidth',5)

    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 robotCurrentPose(3) 1]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
xlim([-3 3])
ylim([-3 3])
    
    vl_command = (floor(phi_l*2095/12));
    vr_command = (floor(phi_r*2095/12));
    robotBmsg.Data = int32([vl_command,vr_command]);
    
    send(robotBpub,robotBmsg);


    waitfor(vizRate);

    
end

    robotBmsg.Data = int32([0,0]);
    
    send(robotBpub,robotBmsg);