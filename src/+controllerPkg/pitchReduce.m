% Main Authors: David Reynolds (dreynolds45@gatech.edu)

function x3 = pitchReduce(x14)
	% Maps full 14-state vector to Pitch states: [Alpha; q; Theta]
	% https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling

    u = x14(5); v = x14(6); w = x14(7);
    q_body = x14(12:14);

    % angle of attack
    alpha = atan2(w, u);

    % pitch from quaternion
    qw = x14(8);
    qx = x14(9);
    qy = x14(10);
    qz = x14(11);
    theta = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2));

    % pitch rate (q)
    q = q_body(2); % body y-axis rate

    x3 = [alpha; q; theta];
end