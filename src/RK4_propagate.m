function x_next = RK4_propagate(x, u, t, dt, eom_fun, thrust_fun)
% One RK4 step for x' = eom_fun(x,u,t,T(t))
% x_next = x + dt * RK4(...)
    T1 = thrust_fun(t);
    k1 = eom_fun(x,              u, t,           T1);

    T2 = thrust_fun(t + 0.5*dt);
    k2 = eom_fun(x + 0.5*dt*k1,  u, t + 0.5*dt, T2);

    k3 = eom_fun(x + 0.5*dt*k2,  u, t + 0.5*dt, T2);

    T4 = thrust_fun(t + dt);
    k4 = eom_fun(x + dt*k3,      u, t + dt,     T4);

    if any(~isfinite([k1; k2; k3; k4]))
        error('RK4 produced non-finite stage at t=%.6f.', t);
    end

    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % Optional: keep quaternion unit length (cheap, helps stability)
    % q = x_next(10:13); qn = norm(q);
    % if qn > 0, x_next(10:13) = q / qn; end
end
