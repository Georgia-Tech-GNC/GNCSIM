% Main Authors: David Reynolds (dreynolds45@gatech.edu)

function x3 = pitchReduce(x14, yaw)
	% Maps full 14-state vector to Pitch states: [Alpha; q; Theta]
	% https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling

    arguments
        x14
        yaw = false;
	end

	if yaw
        %q = Utils.eulerToQuat([90 0 0]);
		%DCM = Utils.dirCosine(q);

		q0 = x14(8);
    	q1 = x14(9);
    	q2 = x14(10);
    	q3 = x14(11);

		quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                quat_x90 = quaternion(Utils.eulerToQuat([-90, 0, 0])');
                quat_rotated = quat * quat_x90;

		DCM = Utils.dirCosine(compact(quat_x90));
		x14(8:11) = compact(quat_rotated);
		x14(5:7) = DCM * x14(5:7);
		x14(12:14) = DCM * x14(12:14);
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
 
    % Extract Pitch (Theta) - Default
    % Note: Using asin avoids singularity issues better than atan2 for pitch
    theta = asin(2*(qw_rot*qy_rot - qx_rot*qz_rot));


	% pitch rate (q) - Default
	q = q_body(2); 

    x3 = [alpha; q; theta];
end