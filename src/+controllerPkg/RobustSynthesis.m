

classdef RobustSynthesis < handle
    properties
        rocketParams
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
        function obj = RobustSynthesis(rocketParams)

            obj.rocketParams = rocketParams;

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
            
            % Maybe we could change the values in a better place.
            % Perhaps inside the compute_gains.m loop
            flight.rocketDynamics.setUncertainParameters(massWet, massDry, ...
                diag([flight.rocketDynamics.moiWet(1,1), moiWet, moiWet]), ...
                diag([flight.rocketDynamics.moiDry(1,1), moiDry, moiDry]), ...
                cgWet, cgDry);
            
            flight.rocketDynamics.clearUncertainParameters();
            
            C = [0 0 1];
            D = 0;
            
            sysPlant = ss(A, B, C, D);
            sysPlant.StateName = {'angle', 'angleDot', 'alpha'};
            sysPlant.OutputName = {'angle'};
            sysPlant.InputName = 'u';

            % Define PID structure
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

            gains = struct();
            gains.Kp = Kp_val;
            gains.Ki = Ki_val;
            gains.Kd = Kd_val;
            gains.Td = Td_val;
            gains.mu = muInfo.MussvBnds(end, 1);
            gains.K_musyn = CL_mu;
            obj.synthesizedGains = gains;
        end
    end
end
   
