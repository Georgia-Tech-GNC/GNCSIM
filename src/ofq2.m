function [omega_cmd, q_goal_used] = ofq2(x, q_goal)
% omega_cmd drives attitude to q_goal with quaternion error feedback.
% x(7:9) = [p;q;r] (body rates)
% x(10:13) = [q0;q1;q2;q3] (your convention)

k_att = 6.0;
k_damp = 1.5;

q = x(10:13);
q = q / norm(q);

% Ensure shortest path (same hemisphere)
if dot(q, q_goal) < 0
    q_goal = -q_goal;
end
q_goal_used = q_goal;

% Quaternion error: q_err = q_goal ⊗ conj(q)
q_conj = [q(1); -q(2); -q(3); -q(4)];

q_err = quatMultiply(q_goal, q_conj);

% Error vector part (axis * sin(angle/2))
e = q_err(2:4);

% Simple PD in body rates
omega = x(7:9);
omega_cmd = 2*k_att*e - k_damp*omega;

end

function q = quatMultiply(a,b)
% Hamilton product a ⊗ b, both as [w;x;y;z]
aw=a(1); ax=a(2); ay=a(3); az=a(4);
bw=b(1); bx=b(2); by=b(3); bz=b(4);

q = [ aw*bw - ax*bx - ay*by - az*bz;
      aw*bx + ax*bw + ay*bz - az*by;
      aw*by - ax*bz + ay*bw + az*bx;
      aw*bz + ax*by - ay*bx + az*bw ];
end
