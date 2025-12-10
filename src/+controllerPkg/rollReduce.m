function x2 = rollReduce(x14)
    % Maps full 14-state vector to Roll states: [Phi; p]
    
    % Roll rate (p) - Body frame rate is typically used directly
    p = x14(12); 

    qw = x14(8);
    qx = x14(9);
    qy = x14(10);
    qz = x14(11);
    
    %{
    c = cos(pi/4);
    s = sin(pi/4);

    % Manual Quaternion Multiplication:
    qw_rot = c*qw - s*qy;
    qx_rot = c*qx - s*qz;
    qy_rot = c*qy + s*qw;
    qz_rot = c*qz + s*qx;
    %}

    % Using standard ZYX roll formula on the new components
    phi = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    x2 = [p; phi];
end