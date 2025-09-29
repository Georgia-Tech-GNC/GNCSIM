function [A_num, B_num] = linearize()



    T = [0.0, 0.457288;
        0.152, 0.314574;
        0.162, 40.2069;
        0.164, 148.3954;
        0.168, 598.904;
        0.172, 890.058;
        0.176, 995.096;
        0.178, 1158.031;
        0.182, 1342.306;
        0.19, 1556.716;
        0.222, 1959.8;
        0.234, 2050.53;
        0.254, 2111.01;
        0.308, 2119.9;
        0.356, 2073.29;
        0.47, 2091.07;
        0.528, 1989.543;
        0.582, 1825.825;
        0.648, 1733.526;
        0.782, 1658.07;
        0.892, 1701.725;
        0.914, 1633.231;
        0.966, 1669.558;
        1.7839, 1594.076;
        1.8239, 1551.156;
        3.1999, 1533.43;
        5.4538, 1378.986;
        6.9577, 1170.933;
        10.9216, 424.692;
        13.5895, 106.0499;
        14.6754, 12.60914;
        16.1354, 0.0;];
        
        valid_indices = find(T(:,1)<=time_elapsed);
        closest_index = max(valid_indices);  % Get the last valid index
        cT = T(closest_index,2);  %Current Thrust THIS NEEDS TO BE MADE BETTER
    
        wetMOI = [0.2109 0 0; 0 21.1212 0; 0 0 21.1202];
        dryMOI = [0.2036 0 0; 0 18.99 0; 0 0 18.988];
        wetMass = 38.87;
        dryMass = 30.96;
        finalCG = 2.34;
        CG0 = 2.433;
        burnTime = 13.1;
        Ib = wetMOI - ((wetMOI - dryMOI)/burnTime) .* time_elapsed;
        inv_Ib = inv(Ib);
        m = wetMass - ((wetMass - dryMass)/burnTime) * time_elapsed; %mass
        CG = CG0 - ((CG0 - finalCG)/burnTime) * time_elapsed; %center of gravity
        rho = 0.0023769 .* (1 - 6.877e-6 .* x(1)).^4.25588; %air density
        g = 9.81;
        a = atan(x(5)/x(4)); %angle of attack
        v = sqrt(x(4)^2 + x(5)^2 + x(6)^2); %magnitude of velocity
        Lmag = 0.193088*rho*v^2*a / (3.59306 + sqrt(1 - (v/1097.40)^2)) + 1.22718*rho*v^2*asin(x(9)*(10.3264-CG)/v) / (1+sqrt(1+0.797628*(1-(v/1116.45)^2)));
        D_a = 0.46875*rho*v^2*(0.00253*a^2+0.13932);
        Mo = (a .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273)) * 0.6375*rho*v^2;
        MA_b_CG0 = [0;0;Mo];


    
    
       % f = xDot - xDot_var;
       % 
       % E = jacobian(f, xDot);
       % A_prime = jacobian(f, x_var);
       % B_prime = jacobian(f, u);
       % 
       % A = -1*inv(E)*A_prime;
       % B = -1*inv(E)*B_prime;
       % 
       % vars = [x_var, u];
       % XU = [x0; u0];
       % A_num = double(subs(A, vars, XU));
       % B_num = double(subs(B, vars, XU));
end
   