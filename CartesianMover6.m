function [configSoln] = CartesianMover6(X, Y, Z, A, B, C)
    % 1. SETUP ROBOT AND SOLVER
    % We use 'try-catch' to avoid reloading the robot if it's already in memory (faster)
    persistent mover6 ik initialguess weights
    if isempty(mover6)
        mover6 = importrobot('CPMOVER6.urdf');
        ik = inverseKinematics('RigidBodyTree', mover6);
        weights = [0.25 0.25 0.25 1 1 1];
        initialguess = mover6.homeConfiguration;
    end

    % 2. CREATE THE TRANSFORMATION MATRIX (XYZABC Order)
    
    % --- Translations ---
    Tx = trvec2tform([X, 0, 0]);
    Ty = trvec2tform([0, Y, 0]);
    Tz = trvec2tform([0, 0, Z]);
    
    % --- Rotations ---
    % Inputs are in degrees, so we convert to radians.
    % A = Rotation about Z
    Ta = eul2tform([deg2rad(A), 0, 0]); 
    
    % B = Rotation about Y
    Tb = eul2tform([0, deg2rad(B), 0]); 
    
    % C = Rotation about X
    Tc = eul2tform([0, 0, deg2rad(C)]); 
    
    % 3. MULTIPLY IN ORDER (XYZABC)
    % This creates the final Target Matrix
    targetTform = Tx * Ty * Tz * Ta * Tb * Tc;
    
    % Debug: Display the matrix so you can verify it
    disp('Target Matrix:');
    disp(targetTform);

    % 4. SOLVE INVERSE KINEMATICS
    % We ask the solver to find the joint angles that match this matrix
    [configSoln, solnInfo] = ik('link6', targetTform, weights, initialguess);
    
    % Optional: Check if it actually worked
    if strcmp(solnInfo.Status, 'success')
        disp('IK Status: Success');
    else
        disp(['IK Status: ' solnInfo.Status]);
    end

    % 5. VISUALIZATION
    % (You can comment this out if you want it to run silently)
    figure(1);
    show(mover6, configSoln);
    title(['Solution for X=' num2str(X) ' Y=' num2str(Y) ' Z=' num2str(Z)]);
    hold on;
    plotTransforms(tform2trvec(targetTform), tform2quat(targetTform), 'FrameSize', 0.2);
    hold off;
end
