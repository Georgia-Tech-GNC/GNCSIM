classdef JetVanesActuatorModel < controller_pkg.ActuatorModel

    properties
        location;
        q_ba;
        MT;
        biasAng;
        jetAngs;
        servoAngs;
        sideForces;
        r24;
    end

    methods
        
        function [F_b, M_b] = get_forces_and_moments(obj, controlInput, rocket, CG)
                 r = obj.location - CG;
                 obj.jetAngs = obj.jetAngs + controlInput + obj.biasAng;
                 vaneDefFile = readtable('VaneDeflectionAngles.xlsx');
                 vane_Deflection_Angle = cell2mat(vaneDefFile(2:end, 1));
                 sideForceOnTwoVanes = cell2mat(vaneDefFile(2:end, 2));

                 sideForce(1) = interp1(vane_Deflection_Angle, sideForceOnTwoVanes, obj.jetAngs(1));
                 sideForce(2) = interp1(vane_Deflection_Angle, sideForceOnTwoVanes, obj.jetAngs(2));
                 sideForce(3) = interp1(vane_Deflection_Angle, sideForceOnTwoVanes, obj.jetAngs(3));
                 sideForce(4) = interp1(vane_Deflection_Angle, sideForceOnTwoVanes, obj.jetAngs(4));
                 sideForce = sideForce/2;

                 M_yaw = sideForce(1)*2*r;
                 M_roll = sideForce(2)*2*obj.r24;
                 M_b = [M_roll; 0; M_yaw];
                 F_b =  [0 ; 2*sideForce(1); 0]; % Returns (1) Side Force from Vanes 1 & 3 (2) Side Force from Vanes 2 & 4 
                                                        % 2 and 4 is roll
                                                        % 1 and 3 is yaw
        end
        

        % %Getters
        % function [thrustMoment] = get_MT(obj)
        %     thrustMoment = obj.MT;
        % end
        % 
        % function [bias] = get_biasAng(obj)
        %     bias = obj.biasAng;
        % end
        % 
        % function [jetAngs] = get_jetAngs(obj)
        %     jetAngs = obj.jetAngs;
        % end
        % 
        % function [servoAngs] = get_servoAngs(obj)
        %     servoAngs = obj.servoAngs;
        % end
        % 
        % function [sideForces] = get_sideForces(obj)
        %     sideForces = obj.sideForces;
        % end
        % 
        % %Setters
        % function obj = set_biasAng(obj,newBiasAng)
        %     obj.biasAng = newBiasAng;
        % end
    end
end