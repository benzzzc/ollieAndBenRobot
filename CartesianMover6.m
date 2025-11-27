function CartesianMover6(target_x, target_y, target_z, target_A, target_B, target_C)
    % 1. SETUP ROBOT AND SOLVER
    % Load the robot model
    try
        mover6 = importrobot('CPMOVER6.urdf'); 
    catch
        error('Could not load robot. Make sure CPMOVER6.urdf is in your MATLAB path.');
    end
    
    ik = inverseKinematics('RigidBodyTree', mover6);
    weights = [0.25 0.25 0.25 1 1 1]; 
    initialguess = mover6.homeConfiguration;

    % 2. CREATE THE TRANSFORMATION MATRIX
    % We need to calculate one matrix for each component and multiply them 
    % in the order X -> Y -> Z -> A -> B -> C.
    
    % --- Translations (using trvec2tform) ---
    Tx = trvec2tform([target_x, 0, 0]);
    Ty = trvec2tform([0, target_y, 0]);
    Tz = trvec2tform([0, 0, target_z]);
    
    % --- Rotations (using eul2tform) ---
    % Note: eul2tform uses 'ZYX' order by default: [Z-angle, Y-angle, X-angle]
    
    % A = Rotation about Z (Yaw)
    % We put the angle in the 1st slot (Z) and leave others 0
    Ta = eul2tform([deg2rad(target_A), 0, 0]); 
    
    % B = Rotation about Y (Pitch)
    % We put the angle in the 2nd slot (Y)
    Tb = eul2tform([0, deg2rad(target_B), 0]); 
    
    % C = Rotation about X (Roll)
    % We put the angle in the 3rd slot (X)
    Tc = eul2tform([0, 0, deg2rad(target_C)]); 
    
    % 3. MULTIPLY IN ORDER (XYZABC)
    targetTform = Tx * Ty * Tz * Ta * Tb * Tc;
    
    % --- OUTPUTS ---
    disp('-----------------------------------------');
    disp('1. Final Transformation Matrix (Goal):');
    disp(targetTform);
    
    % 4. SOLVE INVERSE KINEMATICS
    [configSoln, solnInfo] = ik('link6', targetTform, weights, initialguess);
    
    disp('2. IK Solver Status:');
    disp(solnInfo.Status);
    
    disp('3. Calculated Joint Angles:');
    % Extract just the numbers for easy reading
    jointAngles = [configSoln.JointPosition];
    disp(jointAngles);
    
    % 5. VISUALIZE
    figure(1);
    show(mover6, configSoln);
    title(['Solution for X=' num2str(target_x) ' Y=' num2str(target_y) ' Z=' num2str(target_z)]);
    hold on;
    % Plot the goal frame to verify the robot reached it
    plotTransforms(tform2trvec(targetTform), tform2quat(targetTform), 'FrameSize', 0.2);
    hold off;
end
