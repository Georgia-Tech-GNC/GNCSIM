uncertainSystem = true;

if uncertainSystem
    u0 = ureal('u0', 1622, 'Percentage', 5); % body velocity
    mua = ureal('mua', 3.4581, 'Percentage', 10); % Aerodynamic coeff
    muc = ureal('muc', -12.98, 'Percentage', 10); % Control coeff
    m = ureal('m', 50, 'Percentage', 5); % vehicle mass
else
    u0 = 1622; % body velocity
    mua = 3.4581; % Aerodynamic coeff
    muc = -12.98; % Control coeff
    m = 50; % vehicle mass
end

% Esto son x_optimal y u_optimal
theta0 = 3.25*(pi/180);
g = -9.807;
La = 3.06558e+03;
Za = La/m;

Ap = [0, 1, 0;
    0, 0, mua/u0;
    -g*sin(theta0), u0, -Za/u0];

Bp = [0;
    muc;
    -107.1];

Cp = [1 0 0;
      0 1 0;
      0 0 1];

Dp = [0;
      0;
      0];
% Define the state-space system
sysPlant = ss(Ap, Bp, Cp, Dp);
sysPlant.StateName = {'theta', 'thetaDot', 'Vz'};
sysPlant.OutputName = {'theta', 'thetaDot', 'Vz'};
sysPlant.InputName = 'u';


%% Weighting functions

% Tracking error (esto es Ws)
DC = 1.5;
wc = 1;
HF = 0.01;
We = makeweight(DC, wc, HF);
We.InputName = 'e';
We.OutputName = 'z1';

% (Wu) Weighting function to impose upper bound on control sensitivity function
Wu = tf(0.1);
Wu.InputName = 'u';
Wu.OutputName = 'z2';

% W. f. to impose upper bound con complementary sensitivity function
Wtheta = makeweight(0.1, 25, 5);
Wtheta.InputName = 'theta';
Wtheta.OutputName = 'z3';

% Penalize pitch rate (q) (example of how you can penalize via your z
% however you want
Wq = tf(0.02);
Wq.InputName = 'thetaDot';
Wq.OutputName = 'z4';

%% Disturbance, noise and reference weights
Wd = tf(1e-3);
Wd.InputName = 'dTilde';
Wd.OutputName = 'd';

% Assume white noise input nTilde. Wn acts as a filter to shape noise
Wn = tf(0.02);
Wn.InputName = 'nTilde';
Wn.OutputName = 'n';

% Weight to the reference signal
Wr = tf(1);
Wr.InputName = 'rTilde';
Wr.OutputName = 'r';

%% Controller
% Tuneable PID controller - Kp, Ki, Kd, Td
% realp is just telling Matlab that this parameters are allowed to be tuned
Kp = realp('Kp', 1);
Ki = realp('Ki', 1); % Integral gain
Kd = realp('Kd', 1); % Derivative gain
Td = realp('Td', 0.01); % Derivative time constant

% PID with filtered derivative - Kd*s / (Td*s + 1)
s = tf('s'); 
C_PID = pid(Kp, Ki, Kd, Td);
C_PID.InputName = 'e';
C_PID.OutputName = 'uPID';

% Summing juntions ( defines el loop parece )
sum1 = sumblk('e = r - yTrack'); % error
sum2 = sumblk('yTrack = theta + n'); % meauserement
sum3 = sumblk('u = uPID + d'); % control + disturbance

%% Generalized Plant Tzw
P_uncertain = connect(sysPlant, C_PID,...
    Wu, We, Wtheta, Wq, Wd, Wn, Wr,...
    sum1, sum2, sum3,...
    {'rTilde', 'dTilde', 'nTilde'}, ...
    {'z1', 'z2', 'z3', 'z4'});

%% μ-synthesis
opts = musynOptions('Display','short', 'MaxIter', 5);

% musyn requiere separar las incertidumbres
% Necesitas extraer la parte nominal y las incertidumbres
[CL_mu, muInfo] = musyn(P_uncertain, opts);

% Extraer ganancias del controlador sintetizado
Kp_mu = getBlockValue(CL_mu, 'Kp');
Ki_mu = getBlockValue(CL_mu, 'Ki');
Kd_mu = getBlockValue(CL_mu, 'Kd');
Td_mu = getBlockValue(CL_mu, 'Td');

%disp(['μ upper bound: ', num2str(muInfo.MussvBnds(end,1))]);
disp('Control gains (μ-synthesis):');
disp(['Kp: ', num2str(Kp_mu)]);
disp(['Ki: ', num2str(Ki_mu)]);
disp(['Kd: ', num2str(Kd_mu)]);
disp(['Td: ', num2str(Td_mu)]);

%% Evaluation

% Outputs:
%  z1: We*e (weighted error)
%  z2: Wu*u (weighted control input)
%  z3: Wtheta*theta (weighted pitch output)
%  z4: Wq*thetaDot (weighted pitch rate output)

% Inputs
%  rTilde: (reference input)
%  dTilde: (disturbance to control input)
%  nTilde: (noise on output)

figure(1); clf
step(CL_mu(1,1)*inv(We))
title('Step response From Reference to Error')

figure(2); clf
bode(CL_mu(1,1)*inv(We))
title('Bode Diagram for Closed Loop System From Reference to Error')


figure(3); clf
step(CL_mu(3,1)*inv(Wtheta))
title('Step response From Reference to Pitch Angle')

figure(4); clf
bode(CL_mu(3,1)*inv(Wq))
title('Bode Diagram for Closed Loop System From Reference to Pitch Angle')

figure(5); clf
step(CL_mu(4,1)*inv(Wq))
title('Step response From Reference to Pitch Rate')

figure(6); clf
bode(CL_mu(3,3))















 

