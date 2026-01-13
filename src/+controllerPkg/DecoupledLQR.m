% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef DecoupledLQR < controllerPkg.Controller
    properties
        breakPoints
        referenceAttitudes
        Q2
        Q3
        R2
        R3
        reduceLTI
    end

    methods
        function obj = DecoupledLQR(breakPoints, referenceAttitudes, Q2, Q3, R2, R3, reduceLTI)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.Q2 = Q2;
            obj.Q3 = Q3;
            obj.R2 = R2;
            obj.R3 = R3;
            obj.reduceLTI = reduceLTI;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);

            K2 = interpolatedGains{1};
            K3 = interpolatedGains{2};

            rollControlRef = interpolatedControlReference(1);
            pitchControlRef = interpolatedControlReference(2);
            yawControlRef = interpolatedControlReference(3);

            [rollErr, pitchErr, yawErr] = obj.getError(state, currentTime);

            rollControl = rollControlRef - K2 * rollErr;
            pitchControl = pitchControlRef - K3 * pitchErr;
            yawControl = yawControlRef - K3 * yawErr;

            rocketControlRad = [0, pitchControl, yawControl].';
            rocketControl = rad2deg(rocketControlRad);
        end
           
        function [rollErr, pitchErr, yawErr] = getError(obj, rocketState, currentTime)
            % Rotate orientation by +90 deg pitch to avoid gimbal lock
            q = rocketState(8:11);
            q_pitch90 = [cos(pi/4); 0; sin(pi/4); 0];
            q_new = quatmultiply(q_pitch90.', q.').'; 
            rocketState(8:11) = q_new;

            x3Pitch = obj.reduceLTI.pitchReduce(rocketState);
            x3Yaw = obj.reduceLTI.pitchReduce(rocketState, true);
            x2Roll = obj.reduceLTI.rollReduce(rocketState);

            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));
            
            % Extract reference quaternion at current time
            quat_ref = obj.referenceAttitudes(2:5, idx);
            
            % Convert quaternion to Euler angles
            euler_ref = quat2eul(quat_ref.', 'XYZ');  % Returns [roll, pitch, yaw]
            
            % Extract individual reference angles
            rollRef = euler_ref(1);
            pitchRef = euler_ref(2);
            yawRef = euler_ref(3);
            
            rollErr = [0; rollRef] - x2Roll;
            pitchErr = [0; 0; pitchRef] - x3Pitch;
            yawErr = [0; 0; yawRef] - x3Yaw;
        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight
            for i = 1:obj.breakPoints.numBreakPoints
                time = obj.breakPoints.time(i);
                state = obj.breakPoints.state{i};
                controlInput = obj.breakPoints.controlInput{i};
                actuatorState = obj.breakPoints.actuatorState{i};

                [A3, B3] = obj.reduceLTI.calculateROMpitch(state, controlInput(2), actuatorState, time);
                [A2, B2] = obj.reduceLTI.calculateROMroll(state, controlInput(1), actuatorState, time);

                K3 = lqr(A3, B3, obj.Q3, obj.R3);
                K2 = lqr(A2, B2, obj.Q2, obj.R2);

                obj.breakPoints.stateMatrix{i} = {A2, A3};
                obj.breakPoints.controlMatrix{i} = {B2, B3};
                obj.breakPoints.gain{i} = {K2, K3};               
            end
        end
    end

end