function done = robotA_path(goal)
rosshutdown
rosinit('192.168.1.7')


% parfeval(@robotA_move,1,)

for i=1:length(goal)
goal = goal(i);
robotposeA = rossubscriber("/pos_robA","DataFormat","struct");  
[msg2] = receive(robotposeA,10);
robposA = double(msg2.Data); 

robotposeA_or = rossubscriber("/qualisys/robotA/pose","DataFormat","struct");       
[msg2] = receive(robotposeA_or,10);
robotposeA_orv = double([msg2.Pose.Orientation.X msg2.Pose.Orientation.Y msg2.Pose.Orientation.Z msg2.Pose.Orientation.W]);
robotposeA_orv = rad2deg(quat2eul(robotposeA_orv,'XYZ'));
   
initialOrientation = deg2rad(robotposeA_orv(2)+90);
C_Robot_Pos = [robposA(3) robposA(1)];
C_Robot_Angr = initialOrientation;


goals = ["/pos_st1";"/pos_st2";"/pos_st3";"/pos_st4";"/pos_st5"];
pause_times = [1;2;3;4;4];
goalpose = rossubscriber(goals(goal),"DataFormat","struct");
   [msg2] = receive(goalpose,10);
   goalposed = double(msg2.Data); 
  
pause(1);




% drawbotn([C_Robot_Pos C_Robot_Angr], .1, 1);
% hold on

D_Robot_Pos = [goalposed(3) goalposed(1)];
D_Robot_Angr = 0;
% drawbotn([D_Robot_Pos D_Robot_Angr], .1, 1);

% P controller gains
k_rho = 1;                           %should be larger than 0, i.e, k_rho > 0
k_alpha = 25;                          %k_alpha - k_rho > 0
k_beta = -0.008;                        %should be smaller than 0, i.e, k_beta < 0


d = 0.122;                                 %robot's distance
dt = .1;                                %timestep between driving and collecting sensor data

robotApub = rospublisher("/motorsA","std_msgs/Int32MultiArray","DataFormat","struct");
robotAmsg = rosmessage(robotApub);

goalRadius = 0.3;
distanceToGoal = norm(C_Robot_Pos - D_Robot_Pos);

%%
while( distanceToGoal > goalRadius )
distanceToGoal
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
    vL = v + d/2*w;
    vR = v - d/2*w;
    
    vl_command = (floor(vL*800));
    vr_command = (floor(vR*800));
    robotAmsg.Data = int32([vl_command,vr_command]);
    
    send(robotApub,robotAmsg);
    
    
    
        
   [msg2] = receive(robotposeA,10);
   robposA = double(msg2.Data);
      
    [msg2] = receive(robotposeA_or,10);
   robotposeA_orv = double([msg2.Pose.Orientation.X msg2.Pose.Orientation.Y msg2.Pose.Orientation.Z msg2.Pose.Orientation.W]);
   robotposeA_orv = rad2deg(quat2eul(robotposeA_orv,'XYZ'));
   corientation = deg2rad(robotposeA_orv(2)+90);
   
   posr = [robposA(3);robposA(1);corientation];
    

%     drawbotn(posr, .1, 1);
    C_Robot_Pos = [posr(1) posr(2)];
    C_Robot_Angr = corientation;
    pause(0.01); % if you notice any lagging, try to increase pause time a bit, e.g., 0.05 -> 0.1
    
       distanceToGoal = norm(C_Robot_Pos(:) - D_Robot_Pos(:));

end
    robotAmsg.Data = int32([0,0]);
    
    send(robotApub,robotAmsg);
    pause(pause_times(goal(i)));
    close all
  

end
  done =1 ;
end



%%%%% to plot robot in GUI
% figure,
% plotTrVec = [C_Robot_Pos(1:2), 0];
%     plotRot = axang2quat([C_Robot_Angr*5 0 C_Robot_Angr 1]);
%     plotTransforms(plotTrVec,plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.2,'MeshColor','blue');
%     text(C_Robot_Pos(1)-0.05,C_Robot_Pos(2)+0.2,'Robot A')
% ylim([1 3])
% hold on
% C_Robot_Pos = [1.5,2]
% plotTrVec = [C_Robot_Pos(1:2), 0];
%     plotRot = axang2quat([0 0 C_Robot_Angr 1]);
%     plotTransforms(plotTrVec,plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.2,'MeshColor','green');
%     text(C_Robot_Pos(1)-0.05,C_Robot_Pos(2)+0.2,'Robot B')
