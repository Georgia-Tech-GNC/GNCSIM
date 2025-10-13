function [A_num,B_num] = eval_AB_safe(A,B,x,u,currentThrust,t)
    % Unpack body-axis velocities and r for aero sub-arguments
    ub = x(4); vb = x(5); wb = x(6);
    pb = x(7); qb = x(8); rb = x(9);

    % --- constants (match Jacobian_1) ---
    initial_CG = 2.419762;  final_CG = 2.314814;
    burn_time  = 13.1;

    % Time-varying CG (same as Jacobian_1)
    CG = (t < burn_time) ...
        .* (initial_CG - ((initial_CG - final_CG)/burn_time)*t) ...
        + (t >= burn_time) .* final_CG;

    % --- build safe linearization point (x_lin) ---
    x_lin = x;  % start from true state
    % 1) speed floor to avoid div-by-zero
    V  = sqrt(max(ub*ub + vb*vb + wb*wb, 1e-12));
    % 2) keep Mach < 1 (your aero uses (3.281*V)/1116.45)
    M  = (3.281*V)/1116.45;
    if M >= 0.999
        scale = 0.999 / max(M,5e-4);
        x_lin(4) = ub * scale;
        x_lin(5) = vb * scale;
        x_lin(6) = wb * scale;
        V = sqrt(max(x_lin(4)^2 + x_lin(5)^2 + x_lin(6)^2, 1e-12));
    end
    % 3) keep asin argument strictly inside (-1,1)
    asin_arg = rb*(10.3264 - 3.281*CG) / (3.281*V);
    if asin_arg >= 0.999
        x_lin(9) = rb * (0.999 / max(asin_arg,1e-9));
    elseif asin_arg <= -0.999
        x_lin(9) = rb * (0.999 / max(abs(asin_arg),1e-9));
    end

    % Substitute into symbolic A,B
    names = {'x','y','z','u','v','w','p','q','r','q0','q1','q2','q3', ...
             'u1','u2','u3','T','t'};
    vals  = num2cell([x_lin(:).'  , u(:).' , currentThrust, t]); % 1x18
    A_sub = subs(A, names, vals);
    B_sub = subs(B, names, vals);

    % Numeric + strip tiny imaginary parts
    A_num = double(A_sub);  B_num = double(B_sub);
    tol = 1e-9;
    amax = max(abs(imag(A_num(:)))); bmax = max(abs(imag(B_num(:))));
    if amax > tol, error('A_num has significant imaginary part (max=%.3e).', amax); end
    if bmax > tol, error('B_num has significant imaginary part (max=%.3e).', bmax); end
    A_num = real(A_num);  B_num = real(B_num);
end
