% Main Authors: David Reynolds (dreynolds45@gatech.edu)

function x3 = pitchReduce(x14, yaw)
	% Maps full 14-state vector to Pitch states: [Alpha; q; Theta]
	% https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling

    arguments
        x14
        yaw = false;
    end

    u = x14(5); v = x14(6); w = x14(7);
    q_body = x14(12:14);

    % angle of attack
    alpha = atan2(w, u);

    % pitch from quaternion
    qw = x14(8);
    qx = x14(9);
    qy = x14(10);
    qz = x14(11);

    c = cos(pi/4);
    s = sin(pi/4);
    
    % Manual Quaternion Multiplication: q_new = q * [c, 0, s, 0]
    qw_rot = c*qw - s*qy;
    qx_rot = c*qx - s*qz;
    qy_rot = c*qy + s*qw;
    qz_rot = c*qz + s*qx;
    
    if yaw
        % Extract Yaw (Psi)
        theta = atan2(2*(qw_rot*qz_rot + qx_rot*qy_rot), 1 - 2*(qy_rot^2 + qz_rot^2));
    else
        % Extract Pitch (Theta) - Default
        % Note: Using asin avoids singularity issues better than atan2 for pitch
        theta = asin(2*(qw_rot*qy_rot - qx_rot*qz_rot));
    end

    % pitch rate (q)
    q = q_body(2); 

    x3 = [alpha; q; theta];
end