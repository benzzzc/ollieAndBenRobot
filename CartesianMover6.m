function [configSoln] = CartesianMover6(X,Y,Z,A,B,C)
    %   This function takes the vector form that we input and outputs a
    %   configuration of the mover6 bot that correctly reaches our desired pose
    %  
    %   This function, loads the mover6 model, and initialises the conditions.
    %   it then takes the input arguments for the final pose and calculates the
    %   corresponding transformation matrix. An inverse kinematic solver is
    %   implemeted to calculate the necessary joint angles each joint should
    %   make in order to achieve the desired pose.
    
    %   import the robot urdf, set the kinematics solver, set the home config
    mover6 = importrobot('CPMOVER6.urdf');
    ik = inverseKinematics('RigidBodyTree', mover6);
    weights = [0.25 0.25 0.25 1 1 1];
    initalguess = homeConfiguration(mover6);
    
    %   converts the vector inmputs into seperate transformation matrices
    %   translation
    Tx = trvec2tform([X, 0, 0]);
    Ty = trvec2tform([0, Y, 0]);
    Tz = trvec2tform([0, 0, Z]);
    %   translation = trvec2tform([X, Y, Z,]);
    %   rotation
    Ta = eul2tform([deg2rad(A), 0, 0]);
    Tb = eul2tform([0, deg2rad(B), 0]);
    Tc = eul2tform([0, 0, deg2rad(C)]);
    
    %   create the transform matrix in the format given
    targetTform = Tx * Ty * Tz * Ta * Tb * Tc;
    
    %   ik solver, get the transformation solution 
    %   add solInfo for further info on the solver
    [configSoln] = ik('link6', targetTform, weights, initalguess);
end

function sendRobotMessage(configSoln)
    %   create a publisher node to send message to GoalmovementMover6, 
    %   through the joint_demands topci
    matlabMessageDemandsNode = ros2node("matlab_publisher_node", 0); % change to corresponding domain id 
    demandsPub = ros2publisher(matlabMessageDemandsNode,"/joint_demands","sensor_msgs/msg/JointState");
    demandsMsg = ros2message(demandsPub);
    
    %   translates the data from configSoln to the required data type for a
    %   message, where {} is used to extract names into a cell array and [] is
    %   used to extract the vectors
    demandsMsg.name = {configSoln.JointName};
    demandsMsg.position = [configSoln.JointPosition];
    send(demandsPub, demandsMsg);
    disp('message sent');
end

% in command window
% my_soln = CartesianMover6(Z, Y, Z, A, B, C,) whatevs
% sendRobotMessage(my_soln)

