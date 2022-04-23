function exampleHelperROSPoseCallback_pos(~, message)
    %exampleHelperROSPoseCallback Subscriber callback function for pose data    
    %   exampleHelperROSPoseCallback(~,MESSAGE) returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS message MESSAGE.
    %   
    %   See also ROSPublishAndSubscribeExample.
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    % Declare global variables to store position and orientation
    global pos2
    global orient
    
    % Extract position and orientation from the ROS message and assign the
    % data to the global variables.
    pos2 = [message.pose.position.X message.ose.position.Y message.Linear.Z];
    orient = [message.Angular.X message.Angular.Y message.Angular.Z];
end