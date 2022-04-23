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
   

pause(1);


robotBpub = rospublisher("/motorsB","std_msgs/Int32MultiArray","DataFormat","struct");
robotBmsg = rosmessage(robotBpub);

C_Robot_Pos = [robposB(3) robposB(1)];
C_Robot_Angr = initialOrientation;

D_Robot_Pos = [st1(3) st1(1)];
D_Robot_Angr = 0;

% P controller gains
k_rho = 0.05;                           %should be larger than 0, i.e, k_rho > 0
k_alpha = 0.8;                          %k_alpha - k_rho > 0
k_beta = -0.008;                        %should be smaller than 0, i.e, k_beta < 0


d = 0.122;                                 %robot's distance


%%

goalRadius = 0.3;
distanceToGoal = norm(C_Robot_Pos - D_Robot_Pos);
% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = .2;
% plot(4,6,'*','LineWidth',5)
hold on
while( distanceToGoal > goalRadius )

distanceToGoal 

   [msg2] = receive(robotposeB,10);
   robposB = double(msg2.Data);
   [msg2] = receive(st1pose,10);
   st1 = double(msg2.Data);
   
          
    [msg2] = receive(robotposeB_or,10);
   robotposeB_orv = double([msg2.Pose.Orientation.X msg2.Pose.Orientation.Y msg2.Pose.Orientation.Z msg2.Pose.Orientation.W]);
   robotposeB_orv = rad2deg(quat2eul(robotposeB_orv,'XYZ'));
    corientation = deg2rad(robotposeB_orv(2));

    
    

        C_Robot_Pos = [robposB(3);robposB(1);corientation];
        
        delta_x = D_Robot_Pos(1) - C_Robot_Pos(1);
    delta_y = D_Robot_Pos(2) - C_Robot_Pos(2);
    rho = sqrt(delta_x^2+delta_y^2);    %distance between the center of the robot's wheel axle and the goal position.
    alpha = -C_Robot_Angr+atan2(delta_y,delta_x); %angle between the robot's current direction and the vector connecting the center of the axle of the sheels with the final position.
    
    %limit alpha range from -180 degree to +180
    if rad2deg(alpha) > 180
        temp_alpha = rad2deg(alpha) - 360;
        alpha = deg2rad(temp_alpha);
    elseif rad2deg(alpha) < -180
        temp_alpha = rad2deg(alpha) + 360;
        alpha = deg2rad(temp_alpha);
    end
    
    beta = -C_Robot_Angr-alpha;
    
    % P controller
    v = k_rho*rho;
    w = k_alpha*alpha + k_beta*beta;
    vL = v + d/2*w
    vR = v - d/2*w;
    
%         posr = [C_Robot_Pos C_Robot_Angr];
    posr = drive(posr, d, vL, vR, dt, posr(3)); %determine new position
    
%     vel_data(j,:) = [vL vR];    
%     j=j+1;
%      posr = [robposB(3);robposB(1);corientation];
    robotCurrentPose = posr;
    C_Robot_Pos = [posr(1) posr(2)];
    C_Robot_Angr = posr(3);

%     robotCurrentPose
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - D_Robot_Pos(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
%     plot(path(:,1), path(:,2),"k--d")

    hold all
%         plot(4,6,'*','LineWidth',5)

    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 robotCurrentPose(3) 1]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([-3 3])
    ylim([-3 3])
    
    vl_command = (floor(vL*10095))
    vr_command = (floor(vR*10095))
    robotBmsg.Data = int32([vl_command,vr_command]);
    
    send(robotBpub,robotBmsg);


    waitfor(vizRate);

    
end

    robotBmsg.Data = int32([0,0]);
    
    send(robotBpub,robotBmsg);