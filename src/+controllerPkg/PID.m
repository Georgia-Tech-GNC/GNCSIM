classdef PID < controllerPkg.Controller
    properties
        breakPoints
        referenceAttitudes
        errors
        integralError = [0; 0; 0]
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
            obj.robustSynthesis = controllerPkg.RobustSynthesis;
            numTimeSteps = size(referenceAttitudes, 2);
            obj.errors = zeros(numTimeSteps, 3);
        end
        
        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);
        
            gains_roll = interpolatedGains{1};   % [Kp, Ki, Kd, Td] for roll
            gains_pitch = interpolatedGains{2};  % [Kp, Ki, Kd, Td] for pitch and yaw

            Kp_roll = gains_roll(1);
            Ki_roll = gains_roll(2);
            Kd_roll = gains_roll(3);
            Td_roll = gains_roll(4);

            Kp_pitch = gains_pitch(1);
            Ki_pitch = gains_pitch(2);
            Kd_pitch = gains_pitch(3);
            Td_pitch = gains_pitch(4);
                        
            rollControlRef = interpolatedControlReference(1);
            pitchControlRef = interpolatedControlReference(2);
            yawControlRef = interpolatedControlReference(3);
        

            [rollErr, pitchErr, yawErr] = obj.getError(state, currentTime);
            
           
            rollControl = rollControlRef + Kd_roll * rollErr(1) + Kp_roll * rollErr(2) + Ki_roll * rollErr(3);
            pitchControl = pitchControlRef + Kd_pitch * pitchErr(2) + Kp_pitch * pitchErr(3) + Ki_pitch * pitchErr(4);
            yawControl = yawControlRef + Kd_pitch * yawErr(2) + Kp_pitch * yawErr(3) + Ki_pitch * yawErr(4);
        
            rocketControlRad = [rollControl, pitchControl, yawControl].';
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
            dt = time_vec(2) - time_vec(1);
            numTimeSteps = length(time_vec);

            % Extract reference quaternion at current time
            rollRef = obj.referenceAttitudes(2, idx);
            pitchRef = obj.referenceAttitudes(3, idx);
            yawRef = obj.referenceAttitudes(4, idx);
        
            % Angle error relative to reference
            rollErrAngle = rollRef - x2Roll(2);
            pitchErrAngle = pitchRef - x3Pitch(3);
            yawErrAngle = yawRef - x3Yaw(3);
        
            obj.errors(idx,:) = [rollErrAngle, pitchErrAngle, yawErrAngle]; 
        
            if currentTime > 0 && idx > 1

                % Integrate angle error only
                obj.integralError(1) = obj.integralError(1) + dt * rollErrAngle;
                obj.integralError(2) = obj.integralError(2) + dt * pitchErrAngle;
                obj.integralError(3) = obj.integralError(3) + dt * yawErrAngle;
            end

            rollErr = [x2Roll(1); rollErrAngle; obj.integralError(1)];
            pitchErr = [x3Pitch(1); x3Pitch(2); pitchErrAngle; obj.integralError(2)];
            yawErr = [x3Yaw(1); x3Yaw(2); yawErrAngle; obj.integralError(3)];
        
        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight

            % Save nominal parameter values
            massWet_nom = obj.reduceLTI.rocketDynamics.massWet;
            massDry_nom = obj.reduceLTI.rocketDynamics.massDry;
            cgWet_nom = obj.reduceLTI.rocketDynamics.cgWet;
            cgDry_nom = obj.reduceLTI.rocketDynamics.cgDry;
            moiWet_nom = obj.reduceLTI.rocketDynamics.moiWet;
            moiDry_nom = obj.reduceLTI.rocketDynamics.moiDry;

            fprintf('Total breakpoints: %d\n', obj.breakPoints.numBreakPoints);

            % Loop over all breakpoints
            for i = 1:obj.breakPoints.numBreakPoints
                
                fprintf('[%3d/%d] Processing breakpoint at time = %.3f s ...', i, obj.breakPoints.numBreakPoints, obj.breakPoints.time(i));

                % get parameters needed to get reduced order model
                time = obj.breakPoints.time(i);
                state = obj.breakPoints.state{i};
                controlInput = obj.breakPoints.controlInput{i};
                actuatorState = obj.breakPoints.actuatorState{i};
    
                % Convert parameters to ureal 
                massWet_u = ureal('massWet', massWet_nom, 'Percentage', 5);
                massDry_u = ureal('massDry', massDry_nom, 'Percentage', 5);
                cgWet_u = ureal('cgWet', cgWet_nom, 'Percentage', 5);
                cgDry_u = ureal('cgDry', cgDry_nom, 'Percentage', 5);
                moiWet_long_u = ureal('moiWet_long', moiWet_nom(1,1), 'Percentage', 5);
                moiWet_rot_u = ureal('moiWet_rot', moiWet_nom(2,2), 'Percentage', 5);
                moiWet_u = diag([moiWet_rot_u, moiWet_long_u, moiWet_long_u]);
                moiDry_long_u = ureal('moiDry_long', moiDry_nom(1,1), 'Percentage', 5);
                moiDry_rot_u = ureal('moiDry_rot', moiDry_nom(2,2), 'Percentage', 5);
                moiDry_u = diag([moiDry_rot_u, moiDry_long_u, moiDry_long_u]);

                % Assign uncertain parameters temporarily
                % obj.reduceLTI.rocketDynamics.massWet = massWet_u;
                % obj.reduceLTI.rocketDynamics.massDry = massDry_u;
                % obj.reduceLTI.rocketDynamics.cgWet = cgWet_u;
                % obj.reduceLTI.rocketDynamics.cgDry = cgDry_u;
                % obj.reduceLTI.rocketDynamics.moiWet = moiWet_u;
                % obj.reduceLTI.rocketDynamics.moiDry = moiDry_u;

                % call reduceLTI to get ROM system (with uncertain parameters)
                [Ap, Bp] = obj.reduceLTI.calculateROMpitch(state, controlInput(2), actuatorState, time, obj.useActuatorModel);
                [Ar, Br] = obj.reduceLTI.calculateROMroll(state, controlInput(1), actuatorState, time, obj.useActuatorModel);

                % Restore nominal parameter values before synthesis
                obj.reduceLTI.rocketDynamics.massWet = massWet_nom;
                obj.reduceLTI.rocketDynamics.massDry = massDry_nom;
                obj.reduceLTI.rocketDynamics.cgWet = cgWet_nom;
                obj.reduceLTI.rocketDynamics.cgDry = cgDry_nom;
                obj.reduceLTI.rocketDynamics.moiWet = moiWet_nom;
                obj.reduceLTI.rocketDynamics.moiDry = moiDry_nom;
                
                % We compute PID gains using mu-synthesis
                gains_pitch = obj.robustSynthesis.musynthesis(Ap, Bp);     
                gains_roll = obj.robustSynthesis.musynthesis(Ar, Br);
                
                Kp_pitch = gains_pitch(1);
                Ki_pitch = gains_pitch(2);
                Kd_pitch = gains_pitch(3);
                Td_pitch = gains_pitch(4);
                
                Kp_roll = gains_roll(1);
                Ki_roll = gains_roll(2);
                Kd_roll = gains_roll(3);
                Td_roll = gains_roll(4);

                % Extract nominal values from uncertain systems
                if isobject(Ap) && isprop(Ap, 'NominalValue')
                    Ap = Ap.NominalValue;
                end
                if isobject(Bp) && isprop(Bp, 'NominalValue')
                    Bp = Bp.NominalValue;
                end
                if isobject(Ar) && isprop(Ar, 'NominalValue')
                    Ar = Ar.NominalValue;
                end
                if isobject(Br) && isprop(Br, 'NominalValue')
                    Br = Br.NominalValue;
                end

                obj.breakPoints.stateMatrix{i} = {Ap, Ar};
                obj.breakPoints.controlMatrix{i} = {Bp, Br};

                % Guardar ganancias en estructura anidada
                obj.breakPoints.gain{i} = {
                    [Kp_pitch, Ki_pitch, Kd_pitch, Td_pitch],
                    [Kp_roll, Ki_roll, Kd_roll, Td_roll]     
                    };
                
                fprintf('PITCH: Kp=%.4f, Ki=%.4f, Kd=%.4f, Td=%.4f\n', Kp_pitch, Ki_pitch, Kd_pitch, Td_pitch);
                fprintf('ROLL:  Kp=%.4f, Ki=%.4f, Kd=%.4f, Td=%.4f\n', Kp_roll, Ki_roll, Kd_roll, Td_roll);
            end
            
        end
    end
end