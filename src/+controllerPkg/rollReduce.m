
function x2 = rollReduce(x14)
    % Maps full 14-state vector to Roll states: [Phi; p]
	
	% Roll angle (phi) from quaternion
    qw = x14(8);
    qx = x14(9);
    qy = x14(10);
    qz = x14(11);
    phi = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    
    % roll rate
    p = x14(12); 

    x2 = [p; phi];
end
