function []= display_Onceler(data)

    figure(1); clf
    plot3(data.x, data.y, -data.z)
    xlabel('X')
    ylabel('Y')
    zlabel('Altitude (m)')
    title('3D Trajectory')

    figure(2); clf
    plot(data.t, atan2(data.v,data.u))
    xlabel('t (s)')
    ylabel('alpha')
    title('Angle of Attack')


    figure(3); clf
    plot(data.t, data.u, data.t, data.v, data.t, data.w)
    xlabel('t (s)')
    ylabel('Velocities (m/s)')
    title('Velocity Components')

    figure(4); clf
    plot(data.t, data.p, data.t, data.q, data.t, data.r)
    xlabel('t (s)')
    ylabel('Angular Velocities ')
    title('AngularVelocity Components')

    figure(5); clf
    plot(data.t, data.q0, data.t, data.q1, data.t, data.q2, data.t, data.q3)
    xlabel('t (s)')
    ylabel('Quaternion')
    title('Quaternion Components')

    figure(6); clf
    plot(data.t(1:end-1), data.beta1a, data.t(1:end-1), data.beta1b, data.t(1:end-1), data.beta2a, data.t(1:end-1), data.beta2b)
    xlabel('t (s)')
    ylabel('Fin tab deflection')
    title('Optimal Controls')

    % figure(4); clf
    % plot(data.t, data.p, data.t, data.q, data.t, data.r)
    % xlabel('t (s)')
    % ylabel('Angular Velocities ')
    % title('AngularVelocity Components')
    % 
    % figure(5); clf
    % plot(data.t, data.q0, data.t, data.q1, data.t, data.q2, data.t, data.q3)
    % xlabel('t (s)')
    % ylabel('Quaternion')
    % title('Quaternion Components')
    % 
    % figure(6); clf
    % plot(data.t, data.FAx, data.t, data.FAy, data.t, data.FAz)
    % xlabel('t (s)')
    % ylabel('Aero Forces')
    % title('Aero Force Components')
    % 
    % figure(7); clf
    % plot(data.t, data.MAx, data.t, data.MAy, data.t, data.MAz)
    % xlabel('t (s)')
    % ylabel('Aero Moments')
    % title('Aero Moment Components')

   