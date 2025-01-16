classdef ActuatorModel
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        MT;
        biasAng;
        jetAngs;
        servoAngs;
        sideForces;
    end

    methods
        function obj = ActuatorModel(MT, Tp, t, biasAng)
            obj.MT = MT;
            obj.Tp = Tp;
            obj.t = t;
            obj.biasAng = biasAng;
            
            % Calculating lever arm
            body_length = 3.4;
            CG_i = 2.433;
            CG_f = 2.34;
            dur = 13.1;
            CG = ((CG_f - CG_i)/dur)*(t) + CG_i;
            r = body_length - CG;
            obj.r = r;
        
            % Force calculations
            r24 = (0.5678)/39.37; %converting in to m
            Mroll = MT(1);
            F2 = Mroll/r24;
            F4 = F2;
        
            Myaw = MT(3);
            Fsideyaw = Myaw/(2*r);
            F1 = Fsideyaw;
            F3 = Fsideyaw;
            sideForces = [F1 F2 F3 F4];
            obj.sideForces = sideForces;
        
            % Jet Vane deflections
            staticPressure = '4.0'; % Options: 6.895, 4.5, 4.0, 3.5, 3.0, 2.5 (x10^6 Pa)
            vaneDefFile = readtable('VaneDeflectionAngles.xlsx','Sheet',staticPressure); % Fixed sheet name
            vdCol = vaneDefFile.Var1(2:end);
            sfCol = (vaneDefFile.Var4(2:end)/100) .* Tp;
            
            jetAngs = zeros(1,4);
            jetAngs(1) = interp1(sfCol,vdCol,sideForces(1)*2,'linear','extrap'); %times 2, data is for 2 vanes
            jetAngs(3) = interp1(sfCol,vdCol,sideForces(1)*2,'linear','extrap');
            jetAngs(2) = interp1(sfCol,vdCol,sideForces(2)*2,'linear','extrap');
            jetAngs(4) = interp1(sfCol,vdCol,sideForces(2)*2,'linear','extrap');
            obj.jetAngs = jetAngs;

            % Servo angles - assumed identical for now
            obj.servoAngs = jetAngs + biasAng;
        end

        %Getters
        function [thrustMoment] = get_MT(obj)
            thrustMoment = obj.MT;
        end

        function [bias] = get_biasAng(obj)
            bias = obj.biasAng;
        end

        function [jetAngs] = get_jetAngs(obj)
            jetAngs = obj.jetAngs;
        end

        function [servoAngs] = get_servoAngs(obj)
            servoAngs = obj.servoAngs;
        end

        function [sideForces] = get_sideForces(obj)
            sideForces = obj.sideForces;
        end

        %Setters
        function obj = set_biasAng(obj,newBiasAng)
            obj.biasAng = newBiasAng;
        end
    end
end