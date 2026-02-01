% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef DecoupledLQR < controllerPkg.Controller
    % DECOUPLEDLQR Represents the controller for pitch, yaw and roll

    properties
        breakPoints                 % breakpoints object holding states 
        referenceAttitudes          % Refrence attitudes in including [time, roll, pitch yaw]
        Q2                          % Q matrix for roll LQR control
        Q3                          % Q matrix for pitch/yaw LQR control
        R2                          % R matrix for roll LQR control
        R3                          % R matrix for pitch/yaw LQR control
        reduceLTI                   % object of reduceLTI class
        useActuatorModel = false    % boolean to check if the actuator model is being used
    end

    methods
        % Constructor
        function obj = DecoupledLQR(breakPoints, referenceAttitudes, Q2, Q3, R2, R3, reduceLTI, useActuatorModel)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.Q2 = Q2;
            obj.Q3 = Q3;
            obj.R2 = R2;
            obj.R3 = R3;
            obj.reduceLTI = reduceLTI;
            obj.useActuatorModel = useActuatorModel;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)

            % Get current gains for roll control (K2) and pitch control
            % (K3)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);
            
            % Extect gains from interpolateGain funcition
            K2 = interpolatedGains{1};
            K3 = interpolatedGains{2};
            
            % Get control refrance
            rollControlRef = interpolatedControlReference(1);
            pitchControlRef = interpolatedControlReference(2);
            yawControlRef = interpolatedControlReference(3);
            
            % Compute error 
            [rollErr, pitchErr, yawErr] = obj.getError(state, currentTime);
            
            % Compute control based on refrance and error
            rollControl = rollControlRef + K2 * rollErr;
            pitchControl = pitchControlRef + K3 * pitchErr;
            yawControl = yawControlRef + K3 * yawErr;
            
            % Get rocket control
            rocketControlRad = [0, pitchControl, yawControl].';  % roll, pitch, yaw
            rocketControl = rad2deg(rocketControlRad);
        end
           
        function [rollErr, pitchErr, yawErr] = getError(obj, rocketState, currentTime)

            % Get reduced state from full state
            x3Pitch = obj.reduceLTI.pitchReduce(rocketState);
            x3Yaw = obj.reduceLTI.pitchReduce(rocketState, true);
            x2Roll = obj.reduceLTI.rollReduce(rocketState);

            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));
            
            % Extract reference quaternion at current time
            rollRef = obj.referenceAttitudes(2, idx);
            pitchRef = obj.referenceAttitudes(3, idx);
            yawRef = obj.referenceAttitudes(4, idx);
            
            % compute error based on reference Attitudes
            rollErr = [0; rollRef] - x2Roll;
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
                [A2, B2] = obj.reduceLTI.calculateROMroll(state, controlInput(1), actuatorState, time, obj.useActuatorModel);
				
                % This is used to clean up sytem matrix. Last column should
                % be all zeros and [1,2] should be 1
				A3(:,3) = [0, 0, 0].';
				A3(1,2) = 1;

                % Compute LQR gains
                K3 = lqr(A3, B3, obj.Q3, obj.R3);
                K2 = lqr(A2, B2, obj.Q2, obj.R2);
                
                % save gains and system
                obj.breakPoints.stateMatrix{ii} = {A2, A3};
                obj.breakPoints.controlMatrix{ii} = {B2, B3};
                obj.breakPoints.gain{ii} = {K2, K3};
            end
        end
    end

end