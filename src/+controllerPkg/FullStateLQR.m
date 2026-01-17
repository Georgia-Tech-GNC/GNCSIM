% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef FullStateLQR < controllerPkg.Controller
    properties
        breakPoints
        referenceAttitudes
        Q
        R
        useActuatorModel = false
        rocketDynamics
    end

    methods
        function obj = FullStateLQR(rocketDynamics, breakPoints, referenceAttitudes, Q, R, useActuatorModel)
            obj.rocketDynamics = rocketDynamics;
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.Q = Q;
            obj.R = R;
            obj.useActuatorModel = useActuatorModel;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);
            K = interpolatedGains{1};
            error = obj.getError6(state, currentTime);
            rocketControl = interpolatedControlReference - K * error;
        end
           
        function error = getError6(obj, rocketState, currentTime)
            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, index] = min(abs(time_vec - currentTime));

            quatRef = obj.referenceAttitudes(2:5, index);
            quatRef = quatRef(:);
            quatState = rocketState(obj.idx.quat); 

            LquatRef = Utils.quatLeftMultiplyMatrix(quatRef);
            LquatRefInv = LquatRef';
            quatError = LquatRefInv * quatState;
            quatError = quatError / norm(quatError);

            error = zeros(6,1);
            error(1:3) = quatError(2:4);
            error(4:6) = rocketState(obj.idx.omega);
        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight
            for i = 1:obj.breakPoints.numBreakPoints
                time = obj.breakPoints.time(i);
                state = obj.breakPoints.state{i};
                controlInput = obj.breakPoints.controlInput{i};
                actuatorState = obj.breakPoints.actuatorState{i};

                [A, B] = obj.reduceState6(state, actuatorState, controlInput, time);

                % controllerPkg.Controller.checkControllability(A, B)

                K = lqr(A, B, obj.Q, obj.R);

                obj.breakPoints.stateMatrix{i} = {A};
                obj.breakPoints.controlMatrix{i} = {B};
                obj.breakPoints.gain{i} = {K};
            end
        end

        function [A6, B6] = reduceState6(obj, state, actuatorState, controlInput, time)
            disturbances.wind = [0, 0, 0].';

            % Obtaining A14 and B14
            if obj.useActuatorModel
		        % Define function handles for numerical differentiation
                f = @(x, deflection) obj.rocketDynamics.rocketModel(x, deflection, disturbances, time, [], true, true);
                h = @(u, deflection) obj.rocketDynamics.actuatorModel.actuatorDynamics(u, deflection);
                fx = @(x) f(x, actuatorState);
                fdel = @(del_in) f(state, del_in);
                hu = @(u_in) h(u_in, actuatorState);
    
		        % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, state); % (14x14)
                B_aero = Utils.numericalJacobian(fdel, actuatorState); % (14x4)
                B_actuator = Utils.numericalJacobian(hu, controlInput); % (4x3)
                B = B_aero * B_actuator; % (14x3)
            else
                % Define function handles for numerical differentiation
                f = @(x, u) obj.rocketDynamics.rocketModelControlWrapper(x, u, disturbances, time, [], true, true);
                fx = @(x) f(x, controlInput);
                fu = @(u_in) f(state, u_in);
    
		        % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, state); % (14x14)
                B = Utils.numericalJacobian(fu, controlInput); % (14x3)
            end

            % Removing mass from the state
            g = @(x) obj.reduceMassPosVel(x);
		    
		    % Reduction mapping matrix (13x14)
            Jg = Utils.numericalJacobian(g, state);

		    % Pseudo-inverse for projection (14x13)
            T = pinv(Jg);
		    
		    % Projected System Matrix (13x13)
            A7 = Jg*A*T;

		    % Projected Control Matrix (13x3)
            B7 = Jg*B;

            % Build quaternion reduction matrix (maps 3D attitude error to 4D quat error)
            quat = state(obj.idx.quat);
            % RMat = Utils.quatRightMultiplyMatrix(quat);
            L = Utils.quatLeftMultiplyMatrix(quat);
            H = Utils.quatHatMapMatrix();
            G = 0.5 * L * H;  % 4x3 attitude jacobian 
            % G = 0.5 * RMat * H;
            
            % Build E matrix (13x12) that maps reduced state to 13D state
            % Reduced state: [pos(3); vel(3); attitude_error(3); omega(3)] = 12D
            % Output state: [pos(3); vel(3); quat(4); omega(3)] = 13D
            E = zeros(7, 6);
            E(1:4, 1:3) = G;           % Quaternion from attitude error (4x3)
            E(5:7, 4:6) = eye(3);      % Angular velocity (3x3)
            
            % Final projection
            A6 = E' * A7 * E;  % 6x6
            B6 = E' * B7;      % 6x3
        end

        function x7 = reduceMassPosVel(obj, state)
            x7 = state([obj.idx.quat, obj.idx.omega]);
        end
    end

end