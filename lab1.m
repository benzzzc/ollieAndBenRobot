body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = pi/4;
tform = trvec2tform([0.25, 0.25, 0]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base');
%create a new body for task 1
%Joint 2: Revolute joint, at default position of 30 degrees 
% at a distance of 1 unit in the X from joint 1 
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = pi/6;
tform2 = trvec2tform([1, 0, 0]); % 1 to x
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1
%Joint 3: Revolute joint, at a default position of 0 degrees at a distance 
% of 0.6 units in X and -0.1 units in Y from joint 2 with a frame rotation
% of -90 degrees about Z
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = 0;
eul = [-pi/2 0 0];
tform3 = trvec2tform([0.6, -0.1, 0])*eul2tform(eul);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2');
%Joint 4: Revolute joint, at default position of 45 degrees at a 
% distance of 1 unit in the X direction from 
%joint 3  
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.HomePosition = pi/4;
tform4 = trvec2tform([1, 0, 0]);
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3');
%End effector: Connect this to joint 4.
bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body4');
%config = randomConfiguration(robot);
config = homeConfiguration(robot);
tform = getTransform(robot,config,'endeffector','base');

%%task 2d
% %1. Get each individual step
% T_base_b1 = getTransform(robot, config, 'body1', 'base');
% T_b1_b2   = getTransform(robot, config, 'body2', 'body1');
% T_b2_b3   = getTransform(robot, config, 'body3', 'body2');
% T_b3_b4   = getTransform(robot, config, 'body4', 'body3');
% T_b4_end  = getTransform(robot, config, 'endeffector', 'body4');
% 
% % 2. Multiply them manually
% T_manual_calculation = T_base_b1 * T_b1_b2 * T_b2_b3 * T_b3_b4 * T_b4_end;
% 
% % 3. Compare with the automatic function
% T_automatic = getTransform(robot, config, 'endeffector', 'base');
% 
% % 4. Check the difference (Error)
% error = norm(T_manual_calculation - T_automatic);
% disp('Difference between manual chain multiplication and getTransform:');
% disp(error);
