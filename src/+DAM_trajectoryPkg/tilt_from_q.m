function q_goal = tilt_from_q(q_init, deg)
    % scalar-first quaternion, Hamilton product
    a = deg2rad(deg);
    qd = [cos(a/2); 0; sin(a/2); 0];   % rotation about +y by deg

    q_goal = DAM_trajectoryPkg.quatmul(qd, q_init);      % q_goal = q_delta ⊗ q_init
    q_goal = q_goal / norm(q_goal);
end
