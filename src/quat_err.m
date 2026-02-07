function qe = quat_err(q_goal, q_now)
    % scalar-first
    q_now = q_now / norm(q_now);
    q_goal = q_goal / norm(q_goal);

    % enforce shortest path (same thing you do with dot<0)
    if dot(q_goal, q_now) < 0
        q_goal = -q_goal;
    end

    qe = quatmul(q_goal, quatconj(q_now));  % qe = q_goal ⊗ q_now^{-1}
    qe = qe / norm(qe);
end
