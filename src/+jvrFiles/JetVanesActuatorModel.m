classdef JetVanesActuatorModel < controllerPkg.ActuatorModel

    properties
        location;
        qBA;
        MT;
        biasAng;
        jetAngs;
        servoAngs;
        sideForces;
        r24;
    end

    methods
        
        function [fB, mB] = getForcesMoments(obj, controlInput, rocket, cg)
                 r = obj.location - cg;
                 obj.jetAngs = obj.jetAngs + controlInput + obj.biasAng;
                 vaneDefFile = readtable('VaneDeflectionAngles.xlsx');
                 vaneDeflectionAngle = cell2mat(vaneDefFile(2:end, 1));
                 sideForceOnTwoVanes = cell2mat(vaneDefFile(2:end, 2));

                 sideForce(1) = interp1(vaneDeflectionAngle, sideForceOnTwoVanes, obj.jetAngs(1));
                 sideForce(2) = interp1(vaneDeflectionAngle, sideForceOnTwoVanes, obj.jetAngs(2));
                 sideForce(3) = interp1(vaneDeflectionAngle, sideForceOnTwoVanes, obj.jetAngs(3));
                 sideForce(4) = interp1(vaneDeflectionAngle, sideForceOnTwoVanes, obj.jetAngs(4));
                 sideForce = sideForce/2;

                 mYaw = sideForce(1)*2*r;
                 mRoll = sideForce(2)*2*obj.r24;
                 mB = [mRoll; 0; mYaw];
                 fB =  [0 ; 2*sideForce(1); 0]; % Returns (1) Side Force from Vanes 1 & 3 (2) Side Force from Vanes 2 & 4
                                                        % 2 and 4 is roll
                                                        % 1 and 3 is yaw
        end
        

        % %Getters
        % function [thrustMoment] = getMT(obj)
        %     thrustMoment = obj.MT;
        % end
        %
        % function [bias] = getBiasAng(obj)
        %     bias = obj.biasAng;
        % end
        %
        % function [jetAngs] = getJetAngs(obj)
        %     jetAngs = obj.jetAngs;
        % end
        %
        % function [servoAngs] = getServoAngs(obj)
        %     servoAngs = obj.servoAngs;
        % end
        %
        % function [sideForces] = getSideForces(obj)
        %     sideForces = obj.sideForces;
        % end
        %
        % %Setters
        % function obj = setBiasAng(obj, newBiasAng)
        %     obj.biasAng = newBiasAng;
        % end
    end
end