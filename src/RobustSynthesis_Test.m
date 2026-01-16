clear all;
close all; clc;

moiDryNominal = 0.56;
moiWetNominal = 0.61;
massDryNominal = 3.12;
massWetNominal = 3.32;
cgXWet = 0.71;

S = 0.03;
rho = 1.225;
Cn_alpha = 1;
F = 0.01;

masses = [3.20, 3.15, 3.10];
velocities = [40, 120, 200];
caseNames = {'Break Point 1', 'Break Point 2', 'Break Point 3'};

nCases = length(masses);
results = struct('mass', {}, 'vx', {}, 'Kp', {}, 'Ki', {}, 'Kd', {}, 'Td', {}, 'mu', {}, 'success', {});

for i = 1:nCases
    mass = masses(i);
    vx = velocities(i);
    
    fprintf('TEST CASE %d: %s\n', i, caseNames{i});
    fprintf('Mass: %.2f kg, Velocity: %.1f m/s\n', mass, vx);
    
    moiDry = ureal('moiDry', moiDryNominal, 'Percentage', 5);
    moiWet = ureal('moiWet', moiWetNominal, 'Percentage', 5);
    massDry = ureal('massDry', massDryNominal, 'Percentage', 5);
    massWet = ureal('massWet', massWetNominal, 'Percentage', 5);
    
    Iy = moiDry + (mass - massDry)/(massWet - massDry) * (moiWet - moiDry);
    
    N_alpha = 0.5 * rho * vx^2 * S * Cn_alpha / (mass * vx);
    M_alpha = cgXWet * N_alpha / Iy;
    
    A = [0 1 0;
        0 0 M_alpha;
        -0.01/(mass*vx) 1 -N_alpha];
    B = [0; 1; 0];
    C = [1 0 0; 0 1 0];
    D = [0; 0];

    G = ss(A, B, C, D);
    G.StateName = {'theta', 'thetaDot', 'alpha'};
    G.OutputName = {'theta', 'thetaDot'};
    G.InputName = 'u';

    Ws = makeweight(1.5, 1, 0.01);
    Ws.InputName = 'e';
    Ws.OutputName = 'z1';
    
    Wu = tf(0.1);
    Wu.InputName = 'u';
    Wu.OutputName = 'z2';
    
    Wtp = makeweight(0.1, 25, 5);
    Wtp.InputName = 'theta';
    Wtp.OutputName = 'z3';
    
    Wtpr = tf(0.02);
    Wtpr.InputName = 'thetaDot';
    Wtpr.OutputName = 'z4';
    
    Wd = tf(1e-3);
    Wd.InputName = 'dTilde';
    Wd.OutputName = 'd';
    
    Wn = tf(0.02);
    Wn.InputName = 'nTilde';
    Wn.OutputName = 'n';
    
    Wr = tf(1);
    Wr.InputName = 'rTilde';
    Wr.OutputName = 'r';

    %% Tunable PID controller
    Kp = realp('Kp', 1);
    Ki = realp('Ki', 1);
    Kd = realp('Kd', 1);
    Td = realp('Td', 0.01);

    s = tf('s');
    C_pid = pid(Kp, Ki, Kd, Td);
    C_pid.InputName = 'e';
    C_pid.OutputName = 'uPID';
    
    sum1 = sumblk('e = r - yTrack');
    sum2 = sumblk('yTrack = theta + n');
    sum3 = sumblk('u = uPID + d');
    
    P = connect(G, C_pid,...
        Ws, Wu, Wtp, Wtpr, Wd, Wn, Wr,...
        sum1, sum2, sum3, ...
        {'rTilde', 'dTilde', 'nTilde'},...
        {'z1', 'z2', 'z3', 'z4'});
    
    try
        opts = musynOptions('Display', 'short', 'MaxIter', 5);
        [CL_mu, muInfo] = musyn(P, opts);
        
        Kp_val = getBlockValue(CL_mu, 'Kp');
        Ki_val = getBlockValue(CL_mu, 'Ki');
        Kd_val = getBlockValue(CL_mu, 'Kd');
        Td_val = getBlockValue(CL_mu, 'Td');

        fprintf('Kp:  %.6e\n', Kp_val);
        fprintf('Ki:  %.6e\n', Ki_val);
        fprintf('Kd:  %.6e\n', Kd_val);
        fprintf('Td:  %.6e\n', Td_val);
        
        results(i).mass = mass;
        results(i).vx = vx;
        results(i).Kp = Kp_val;
        results(i).Ki = Ki_val;
        results(i).Kd = Kd_val;
        results(i).Td = Td_val;
        results(i).success = true;
        
    catch ME
        fprintf('FAILED: %s\n', ME.message);
        results(i).mass = mass;
        results(i).vx = vx;
        results(i).Kp = NaN;
        results(i).Ki = NaN;
        results(i).Kd = NaN;
        results(i).Td = NaN;
        results(i).success = false;
    end
end

fprintf('%12s %10s %10s %12s %12s %12s %12s %10s\n', ...
    'Case', 'Mass (kg)', 'Vx (m/s)', 'Kp', 'Ki', 'Kd', 'Td', 'mu');

for i = 1:nCases
    fprintf('%12s %10.2f %10.1f %12.6e %12.6e %12.6e %12.6e %10.4f\n', ...
        caseNames{i}, results(i).mass, results(i).vx, results(i).Kp, ...
        results(i).Ki, results(i).Kd, results(i).Td, results(i).mu);
end

