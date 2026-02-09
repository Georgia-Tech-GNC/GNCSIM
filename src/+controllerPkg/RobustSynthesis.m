% Main Authors: Andrés Chiva (achiva3@gatech.edu)

classdef RobustSynthesis < handle
    properties
        Wu
        Ws
        Wtp
        Wtpr
        Wd
        Wn
        Wr
        synthesizedGains
    end

    methods
        function obj = RobustSynthesis()

            % makeweight(dcGain, bandwidth, highFreqGain)
            obj.Ws = makeweight(3.0, 1, 0.01);
            obj.Ws.InputName = 'e';
            obj.Ws.OutputName = 'z1';

            obj.Wu = tf(0.01);  
            obj.Wu.InputName = 'u';
            obj.Wu.OutputName = 'z2';
             
            obj.Wtp = makeweight(0.1, 25, 5);
            obj.Wtp.InputName = 'angle';
            obj.Wtp.OutputName = 'z3';

            obj.Wtpr = tf(0.02);
            obj.Wtpr.InputName = 'rate';
            obj.Wtpr.OutputName = 'z4';

            % 
            obj.Wd = tf(1e-3);
            obj.Wd.InputName = 'dTilde';
            obj.Wd.OutputName = 'd';
            
            obj.Wn = tf(0.02);
            obj.Wn.InputName = 'nTilde';
            obj.Wn.OutputName = 'n';
            
            obj.Wr = tf(1);
            obj.Wr.InputName = 'rTilde';
            obj.Wr.OutputName = 'r';

        end

        function gains = musynthesis(obj, A, B)
            
            % Determine system size (2 for roll, 3 for pitch/yaw)
            nStates = size(A, 1);
            
            % C matrix selects the last state (position/angle)
            C = zeros(1, nStates);
            C(nStates) = 1;
            D = 0;
            
            sysPlant = ss(A, B, C, D);
            if nStates == 2
                sysPlant.StateName = {'rate', 'angle'};
            else
                sysPlant.StateName = {'angle', 'rate', 'position'};
            end
            sysPlant.OutputName = {'angle'};
            sysPlant.InputName = 'u';

            % Define PID structure
            s = tf('s');
            Kp = realp('Kp', 1);
            Ki = realp('Ki', 1);
            Kd = realp('Kd', 1);
            Td = realp('Td', 0.01);
            C_PID = pid(Kp, Ki, Kd, Td);
            C_PID.InputName = 'e';
            C_PID.OutputName = 'uPID';

            sum1 = sumblk('e = r - yTrack');
            sum2 = sumblk('yTrack = angle + n');
            sum3 = sumblk('u = uPID + d');

            P_uncertain = connect(sysPlant, C_PID,...
                obj.Ws, obj.Wu, obj.Wtp, obj.Wtpr, obj.Wd, obj.Wn, obj.Wr,...
                sum1, sum2, sum3,...
                {'rTilde', 'dTilde', 'nTilde'},...
                {'z1', 'z2', 'z3', 'z4'});

            opts = musynOptions('Display', 'short', 'MaxIter', 5);
            [CL_mu, muInfo] = musyn(P_uncertain, opts);

            Kp_val = getBlockValue(CL_mu, 'Kp');
            Ki_val = getBlockValue(CL_mu, 'Ki');
            Kd_val = getBlockValue(CL_mu, 'Kd');
            Td_val = getBlockValue(CL_mu, 'Td');

            gains = zeros(1,4);
            gains(1) = Kp_val;
            gains(2) = Ki_val;
            gains(3) = Kd_val;
            gains(4) = Td_val;
            % gains.mu = muInfo.MussvBnds(end, 1);
            % gains.K_musyn = CL_mu;
            obj.synthesizedGains = gains;
        end
    end
end
   
