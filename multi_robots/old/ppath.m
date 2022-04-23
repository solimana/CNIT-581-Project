%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RobotPosCtrl_Sim.m - A mobile robot position control simulator
% this code is modified on 2022-3-3 by BCM
%
% this code shows the single robot's movement based on initial position and
% desired position. 
% Step: click the left button mouse to designate the initial position and
% the right button mouse for the initial heading angle.
% Then click the left button mouse to designate the desired position and 
% the right button mouse for desired heading angle (not implemented version). 
% The robot will move then with P controller. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
   
initialOrientation = deg2rad(robotposeB_orv(2)+90);
% initialOrientation = deg2rad(90);


st1pose = rossubscriber("/pos_st1","DataFormat","struct");
   [msg2] = receive(st1pose,10);
   st1 = double(msg2.Data); 
   

pause(1);


robotBpub = rospublisher("/motorsB","std_msgs/Int32MultiArray","DataFormat","struct");
robotBmsg = rosmessage(robotBpub);

C_Robot_Pos = [robposB(3) robposB(1)];
C_Robot_Angr = initialOrientation;

drawbotn([C_Robot_Pos C_Robot_Angr], .1, 1);
hold on

D_Robot_Pos = [st1(3) st1(1)];
D_Robot_Angr = 0;
drawbotn([D_Robot_Pos D_Robot_Angr], .1, 1);

% P controller gains
k_rho = 0.25;                           %should be larger than 0, i.e, k_rho > 0
k_alpha = 0.8;                          %k_alpha - k_rho > 0
k_beta = -0.008;                        %should be smaller than 0, i.e, k_beta < 0


d = 0.122;                                 %robot's distance
dt = .1;                                %timestep between driving and collecting sensor data

    

%% 
%for vel_data vector count

goalRadius = 0.1;
distanceToGoal = norm(C_Robot_Pos - D_Robot_Pos);


while( distanceToGoal > goalRadius )

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
    vR = v - d/2*w
    

    
    posr = [C_Robot_Pos C_Robot_Angr];
    posr = drive(posr, d, vL, vR, dt, posr(3)); %determine new position
    drawbotn(posr, .1, 1);
    C_Robot_Pos = [posr(1) posr(2)];
    C_Robot_Angr = posr(3);
    pause(0.05); % if you notice any lagging, try to increase pause time a bit, e.g., 0.05 -> 0.1
    
        distanceToGoal = norm(C_Robot_Pos(:) - D_Robot_Pos(:))

end
%%
%for velocity plot
figure
plot(vel_data);
title('Velocities of Two Wheels');
