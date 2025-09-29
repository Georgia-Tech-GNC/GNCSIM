function [x_new] = RK4_propagate(x,u,A,B,delta_t,dt)

steps = delta_t / dt - 1;



    F1 = A*x + B*u;
    F2 = A*(x + 0.5*dt*F1) + B*u;
    F3 = A*(x + 0.5*dt*F2) + B*u;
    F4 = A*(x + dt*F3)     + B*u;

    if any(~isfinite(F1)) || any(~isfinite(F2)) || any(~isfinite(F3)) || any(~isfinite(F4))
        error('RK4 produced non-finite stage at t-step.');
    end

    x = x + (dt/6) * (F1 + 2*F2 + 2*F3 + F4);


x_new = x;