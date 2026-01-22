classdef PID < controllerPkg.Controller
    properties
        breakPoints
        referenceAttitudes
        Kp
        Ki
        Kd
        Td
        errors
        reduceLTI
        robustSynthesis
        useActuatorModel = false
    end

    methods

        function obj = PID(breakPoints, referenceAttitudes, reduceLTI, useActuatorModel)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.reduceLTI = reduceLTI;
            obj.useActuatorModel = useActuatorModel;
            obj.robustsynthesis = controllerPkg.RobustSynthesis;
        end
        
        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);
        
            gains_roll = interpolatedGains{1};   % [Kp, Ki, Kd, Td] para roll
            gains_pitch = interpolatedGains{2};  % [Kp, Ki, Kd, Td] para pitch
            gains_yaw = interpolatedGains{3};    % [Kp, Ki, Kd, Td] para yaw

            Kp_roll = gains_roll(1);
            Ki_roll = gains_roll(2);
            Kd_roll = gains_roll(3);
            Td_roll = gains_roll(4);

            Kp_pitch = gains_pitch(1);
            Ki_pitch = gains_pitch(2);
            Kd_pitch = gains_pitch(3);
            Td_pitch = gains_pitch(4);
            
            Kp_yaw = gains_yaw(1);
            Ki_yaw = gains_yaw(2);
            Kd_yaw = gains_yaw(3);
            Td_yaw = gains_yaw(4);
                        
            rollControlRef = interpolatedControlReference(1);
            pitchControlRef = interpolatedControlReference(2);
            yawControlRef = interpolatedControlReference(3);
        
            % We compute the error, derivative and integral error for each
            % Err(1) = proportional
            % Err(2) = integral 
            % Err(3) = derivative
            [rollErr, pitchErr, yawErr] = obj.getError(state, currentTime);
            
            % I am missing the filter derivative part
            rollControl = rollControlRef + Kp_roll * rollErr + Ki_roll * rollErr(2) + Kd_roll * rollErr(3) ;
            pitchControl = pitchControlRef + Kp_pitch * pitchErr(1) + Ki_pitch * pitchErr(2) + Kd_pitch * pitchErr(3);
            yawControl = yawControlRef + Kp_yaw * yawErr(1) + Ki_yaw * yawErr(2) + Kd_yaw * yawErr(3);
        
            rocketControlRad = [0, pitchControl, yawControl].';
            rocketControl = rad2deg(rocketControlRad);
        end
        
        function [rollErr, pitchErr, yawErr] = getError(obj, rocketState, currentTime)
            x3Pitch = obj.reduceLTI.pitchReduce(rocketState);
            x3Yaw = obj.reduceLTI.pitchReduce(rocketState, true);
            x2Roll = obj.reduceLTI.rollReduce(rocketState);
        
            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));
            dt = time_vec(2) - time_vec(1);
            
            % Extract reference quaternion at current time
            rollRef = obj.referenceAttitudes(2, idx);
            pitchRef = obj.referenceAttitudes(3, idx);
            yawRef = obj.referenceAttitudes(4, idx);
        
            rollErr(1) = [0; rollRef] - x2Roll;
            pitchErr(1) = [0; 0; pitchRef] - x3Pitch;
            yawErr(1) = [0; 0; yawRef] - x3Yaw;
        
            obj.errors(idx,:) = [rollErr(1), pitchErr(1), yawErr(1)]; 
        
            if currentTime>0
                % The integral error is just the addition of rectangles
                % Need to check this
                rollErr(2) = sum(dt,obj.errors(1:idx,1));
                pitchErr(2) = sum(dt,obj.errors(1:idx,2));
                yawErr(2) = sum(dt,obj.errors(1:idx,3));
        
                rollErr(3) = (obj.errors(idx,1) - obj.errors(idx-1,1)) / dt;
                pitchErr(3) = (obj.errors(idx,2) - obj.errors(idx-1,2)) / dt;
                yawErr(3) = (obj.errors(idx,3) - obj.errors(idx-1,3)) / dt;
            else
                rollErr(2) = 0;
                pitchErr(2) = 0;
                yawErr(2) = 0;
                rollErr(3) = 0;
                pitchErr(3) = 0;
                yawErr(3) = 0;
            end
        
        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight
            for i = 1:obj.breakPoints.numBreakPoints
                time = obj.breakPoints.time(i);
                state = obj.breakPoints.state{i};
                controlInput = obj.breakPoints.controlInput{i};
                actuatorState = obj.breakPoints.actuatorState{i};
    
                % Here we should change the nominal values to ureal
                %%%

                [Ap, Bp] = obj.reduceLTI.calculateROMpitch(state, controlInput(2), actuatorState, time, obj.useActuatorModel);
                [Ay, By] = obj.reduceLTI.calculateROMyaw(state, controlInput(3), actuatorState, time, obj.useActuatorModel);
                [Ar, Br] = obj.reduceLTI.calculateROMroll(state, controlInput(1), actuatorState, time, obj.useActuatorModel);
                
                % We compute PID gains using mu-synthesis
                [Kp_pitch, Ki_pitch, Kd_pitch, Td_pitch] = obj.robustsynthesis.synthesize(Ap, Bp);
                [Kp_yaw, Ki_yaw, Kd_yaw, Td_yaw] = obj.robustsynthesis.synthesize(Ay, By);
                [Kp_roll, Ki_roll, Kd_roll, Td_roll] = obj.robustsynthesis.synthesize(Ar, Br);

                obj.breakPoints.stateMatrix{i} = {Ap, Ay, Ar};
                obj.breakPoints.controlMatrix{i} = {Bp, By, Br};

                % Guardar ganancias en estructura anidada
                obj.breakPoints.gain{i} = {
                    [Kp_roll, Ki_roll, Kd_roll, Td_roll],     
                    [Kp_pitch, Ki_pitch, Kd_pitch, Td_pitch], 
                    [Kp_yaw, Ki_yaw, Kd_yaw, Td_yaw]         
                };            
            end
        end
    end
end