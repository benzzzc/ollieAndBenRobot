function [configSoln] = CartesianMover6(X,Y,Z,A,B,C)
%   This function takes the vector form that we input and outputs a
%   configuration of the mover6 bot that correctly reaches our desired pose
%  
%   Detailed explanation goes here
%   import the robot urdf, set the kinematics solver, set the home config
mover6 = importrobot('CPMOVER6.urdf');
ik = inverseKinematics('RigidBodyTree', mover6);
weights = [0.25 0.25 0.25 1 1 1];
initalguess = homeConfiguration(mover6);

%   Create the transformation matrix from the inputs
%   Translation
Tx = trvec2tform([X, 0, 0]);
Ty = trvec2tform([0, Y, 0]);
Tz = trvec2tform([0, 0, Z]);
%   Rotations
Ta = eul2tform([deg2rad(A), 0, 0]);
Tb = eul2tform([0, deg2rad(B), 0]);
Tc = eul2tform([0, 0, deg2rad(C)]);

%   create the transform matrix in the format given
targetTform = Tx * Ty * Tz * Ta * Tb * Tc;

%   ik solver, get the transformation solution 
[configSoln, solnInfo] = ik('link6', targetTform, weights, initalguess);
end
