classdef RobustSynthesis < handle
    properties
        rocketDynamics
        rocketParams
        aero
        Wu
        Ws
        Wtp
        Wtpr
        synthesizedGains
        defaults
    end

    methods
        function obj = RobustSynthesis(rocketDynamics, rocketParams)
            obj.rocketDynamics = rocketDynamics;
            obj.rocketParams = rocketParams;
            obj.aero = rocketDynamics.aero;

            obj.Ws = makeweight(1.5,1,0.01);
            obj.Ws.InputName = 'e';
            obj.Ws.OutputName = 'z1';

            obj.Wu = tf(0.1);
            obj.Wu.InputName = 'u';
            obj.Wu.OutputName = 'z2';
             
            obj.Wtp = makeweight(0.1,25,5);
            obj.Wtp.InputName = 'theta';
            obj.Wtp.OutputName = 'z3';

            obj.Wtpr = tf(0.02);
            obj.Wtpr.InputName = 'thetaDot';
            obj.Wtpr.OutputName = 'z4';

            obj.defaults.F = 0.01;
            obj.defaults.Cn_alpha = 1;
            obj.defaults.cp = 1;
        end

        function gains = synthesize(obj, x, u)
            mass = x(1);
            vx = x(5);
            
            moiDryVal = obj.rocketParams.longitudinalMoiDry;
            moiWetVal = obj.rocketParams.longitudinalMoiWet;
            massDryVal = obj.rocketParams.massDry;
            massWetVal = obj.rocketParams.massWet;
            cgDryVal = obj.rocketParams.cgXDry;
            cgWetVal = obj.rocketParams.cgXWet;
            
            moiDry = ureal('moiDry', moiDryVal, 'Percentage', 5);
            moiWet = ureal('moiWet', moiWetVal, 'Percentage', 5);
            massDry = ureal('massDry', massDryVal, 'Percentage', 5);
            massWet = ureal('massWet', massWetVal, 'Percentage', 5);
            cgDry = ureal('cgDry', cgDryVal, 'Percentage', 5);
            cgWet = ureal('cgWet', cgWetVal, 'Percentage', 5);
            
            obj.rocketDynamics.setUncertainParameters(massWet, massDry, ...
                diag([obj.rocketParams.rotationalMoiWet, moiWet, moiWet]), ...
                diag([obj.rocketParams.rotationalMoiDry, moiDry, moiDry]), ...
                cgWet, cgDry);
            
            reduceLti = analysisPkg.ReduceLTI(obj.rocketDynamics.flight);
            [A3, B3] = reduceLti.calculateROMpitch(x, u);
            
            obj.rocketDynamics.clearUncertainParameters();
            
            C = [1 0 0];
            D = 0;
            
            sysPlant = ss(A3, B3, C, D);
            sysPlant.StateName = {'theta', 'thetaDot', 'alpha'};
            sysPlant.OutputName = {'theta'};
            sysPlant.InputName = 'u';

            Kp = realp('Kp', 1);
            Ki = realp('Ki', 1);
            Kd = realp('Kd', 1);
            Td = realp('Td', 0.01);
            C_PID = pid(Kp, Ki, Kd, Td);
            C_PID.InputName = 'e';
            C_PID.OutputName = 'uPID';

            Wd = tf(1e-3);
            Wd.InputName = 'dTilde';
            Wd.OutputName = 'd';
            
            Wn = tf(0.02);
            Wn.InputName = 'nTilde';
            Wn.OutputName = 'n';
            
            Wr = tf(1);
            Wr.InputName = 'rTilde';
            Wr.OutputName = 'r';

            sum1 = sumblk('e = r - yTrack');
            sum2 = sumblk('yTrack = theta + n');
            sum3 = sumblk('u = uPID + d');

            P_uncertain = connect(sysPlant, C_PID,...
                obj.Ws, obj.Wu, obj.Wtp, obj.Wtpr, Wd, Wn, Wr,...
                sum1, sum2, sum3, {'rTilde', 'dTilde', 'nTilde'}, {'z1', 'z2', 'z3', 'z4'});

            opts = musynOptions('Display', 'short', 'MaxIter', 5);
            [CL_mu, muInfo] = musyn(P_uncertain, opts);

            Kp_val = getBlockValue(CL_mu, 'Kp');
            Ki_val = getBlockValue(CL_mu, 'Ki');
            Kd_val = getBlockValue(CL_mu, 'Kd');
            Td_val = getBlockValue(CL_mu, 'Td');

            gains = struct();
            gains.Kp = Kp_val;
            gains.Ki = Ki_val;
            gains.Kd = Kd_val;
            gains.Td = Td_val;
            gains.mu = muInfo.MussvBnds(end, 1);
            gains.K_musyn = CL_mu;
            gains.breakPoint = struct('x', x, 'u', u);
            obj.synthesizedGains = gains;
        end
    end
end
   
