mover6=importrobot("CPMOVER6.urdf");
config = homeConfiguration(mover6);
%config = randomConfiguration(mover6);

%task 3a
T_base_l1 = getTransform(mover6, config, 'link1', 'base_link');
T_l1_l2   = getTransform(mover6, config, 'link2', 'link1');
T_l2_l3   = getTransform(mover6, config, 'link3', 'link2');
T_l3_l4   = getTransform(mover6, config, 'link4', 'link3');
T_l4_l5   = getTransform(mover6, config, 'link5', 'link4');
T_l6_l5   = getTransform(mover6, config, 'link6', 'link5');
 
% 2. Multiply them manually
transformPoint = T_base_l1 * T_l1_l2 * T_l2_l3 * T_l3_l4 * T_l4_l5 * T_l6_l5;
disp(transformPoint);

ik=inverseKinematics('RigidBodyTree',mover6);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = mover6.homeConfiguration;
randConfig = randomConfiguration(mover6);
tform = getTransform(mover6, randConfig, 'link6', 'base_link');% you acc need somewhere for the link to go 
[configSoln,solnInfo] = ik('link6',tform,weights,initialguess);
disp('--- Target Transform ---');
disp(tform);
