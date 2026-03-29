% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef DecoupledLQR < controllerPkg.Controller
    % DECOUPLEDLQR Represents the controller for pitch and yaw
    % FOR THE LORAX

    properties
        Q3 (3,3) double                         % Q matrix for pitch/yaw LQR control
        R3 (1,1) double                         % R matrix for pitch/yaw LQR control
        reduceLTI controllerPkg.ReduceLTI       % object of reduceLTI class
    end

    methods
        % Constructor
        function obj = DecoupledLQR(breakPoints, referenceAttitudes, Q3, R3, reduceLTI, useActuatorModel)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.Q3 = Q3;
            obj.R3 = R3;
            obj.reduceLTI = reduceLTI;
            obj.useActuatorModel = useActuatorModel;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            arguments
                obj
                state (:,1) double
                currentCumVelocity (1,1) double {mustBeNonnegative}
                currentTime (1,1) double {mustBeNonnegative}
            end

            % Get current gains for pitch control (K3)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);
            
            % Extect gains from interpolateGain funcition
            K3 = interpolatedGains{2};
            
            % Get control refrance
            pitchControlRef = interpolatedControlReference(2);
            yawControlRef = interpolatedControlReference(3);
            
            % Compute error 
            [pitchErr, yawErr] = obj.getError(state, currentTime);
            
            % Compute control based on reference and error
            pitchControl = pitchControlRef + K3 * pitchErr;
            yawControl = yawControlRef + K3 * yawErr;
            
            % Get rocket control
            rocketControlRad = [pitchControl, yawControl].';  % pitch, yaw
            rocketControl = rad2deg(rocketControlRad);
        end
           
        function [pitchErr, yawErr] = getError(obj, rocketStateEstimate, currentTime)
            arguments
                obj
                rocketStateEstimate (:,1) double
                currentTime (1,1) double {mustBeNonnegative}
            end
            %Estimate and State
            rocketStateEstimate(:,1);

            % Get reduced state from full state
            x3Pitch = obj.reduceLTI.pitchReduce(rocketStateEstimate);
            x3Yaw = obj.reduceLTI.pitchReduce(rocketStateEstimate, true);

            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));
            
            % Extract reference quaternion at current time
            pitchRef = obj.referenceAttitudes(2, idx);
            yawRef = obj.referenceAttitudes(3, idx);
            
            % compute error based on reference Attitudes
            pitchErr = [0; 0; pitchRef] - x3Pitch;
            yawErr = [0; 0; yawRef] - x3Yaw;
        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight

            % Loop over all breakpoints
            for ii = 1:obj.breakPoints.numBreakPoints

                % get parameters needed to get reduced oder model
                time = obj.breakPoints.time(ii);
                state = obj.breakPoints.state{ii};
                controlInput = obj.breakPoints.controlInput{ii};
                actuatorState = obj.breakPoints.actuatorState{ii};
                
                % call reduceLTI to get ROM system
                [A3, B3] = obj.reduceLTI.calculateROMpitch(state, controlInput(2), actuatorState, time, obj.useActuatorModel);

                % This is used to clean up sytem matrix. Last column should
                % be all zeros and [1,2] should be 1
				A3(:,3) = [0, 0, 0].';
				A3(1,2) = 1;

                % Compute LQR gains
                K3 = lqr(A3, B3, obj.Q3, obj.R3);
                
                % save gains and system
                obj.breakPoints.stateMatrix{ii} = {A3};
                obj.breakPoints.controlMatrix{ii} = {B3};
                obj.breakPoints.gain{ii} = {K3};
            end
        end
    end

end