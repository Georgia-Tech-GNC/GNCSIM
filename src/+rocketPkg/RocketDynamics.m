% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)
%               David Reynolds (dreynolds45@gatech.edu)
%               Kandarp Vadia (kvadia3@gatech.edu)
%               Aditya Swami (aswami8@gatech.edu)

classdef RocketDynamics < handle & matlab.mixin.Copyable

    properties (Constant)
        idx = StateIndices
    end

    properties
        body            rocketPkg.RocketBody
        rocketFins      rocketPkg.RocketFins
        motor           rocketPkg.Motor
        activeParachute rocketPkg.ParachuteDynamics
        massWet         (1,1) double {mustBeNonnegative}
        massDry         (1,1) double {mustBeNonnegative}
        moiWet          (3,3) double {mustBeNonnegative}
        moiDry          (3,3) double {mustBeNonnegative}
        cgWet           (1,1) double
        cgDry           (1,1) double

        actuatorModel   controllerPkg.ActuatorModel  = controllerPkg.ActuatorModel.empty
        aerodynamicsCFD rocketPkg.AerodynamicsCFD    = rocketPkg.AerodynamicsCFD.empty
    end

    methods
        function obj = RocketDynamics(body, rocketFins, rocketParams, motor, actuatorModel, firstParachute, aerodynamicsCFD)
            arguments
                body                         % DATCOM
                rocketFins                   % DATCOM
                rocketParams
                motor          rocketPkg.Motor
                actuatorModel = controllerPkg.ActuatorModel.empty
                firstParachute = rocketPkg.ParachuteDynamics.empty     % Head of parachute linked list
                aerodynamicsCFD = rocketPkg.AerodynamicsCFD.empty     % defaults to empty
            end

            obj.body       = body;
            obj.rocketFins = rocketFins;
            obj.motor      = motor;

            obj.massWet = rocketParams.massWet;
            obj.massDry = rocketParams.massDry;

            obj.moiWet = diag([rocketParams.rotationalMoiWet, rocketParams.longitudinalMoiWet, rocketParams.longitudinalMoiWet]);
            obj.moiDry = diag([rocketParams.rotationalMoiDry, rocketParams.longitudinalMoiDry, rocketParams.longitudinalMoiDry]);

            obj.cgWet = rocketParams.cgXWet;
            obj.cgDry = rocketParams.cgXDry;

            obj.actuatorModel   = actuatorModel;
            obj.aerodynamicsCFD = aerodynamicsCFD;
            obj.activeParachute = firstParachute;
        end

        function [currentCg, currentMoi] = getCgMoi(obj, mass)
            % getCgMoi  Linearly interpolate CG [m] and MOI [kg·m²] from mass.
            arguments
                obj
                mass (1,1) double {mustBeNonnegative}
            end

            if ~isnumeric(mass)
                error('Error: Mass must be numeric.')
            end

            if mass <= obj.massDry
                currentCg  = obj.cgDry;
                currentMoi = obj.moiDry;
                return
            end

            % Linear interpolation wrt mass
            frac       = (mass - obj.massDry) / (obj.massWet - obj.massDry);
            currentCg  = obj.cgDry  + frac .* (obj.cgWet  - obj.cgDry);
            currentMoi = obj.moiDry + frac .* (obj.moiWet - obj.moiDry);
        end

        function xDot = rocketModelTrajOptWrapper(obj, state, controlInput, atmoConditions, time, flightState, useGravity, log)
            arguments
                obj
                state        (:,1) double
                controlInput (:,1) double
                atmoConditions (1,1) flightPkg.AtmosphericConditions
                time         (1,1) double {mustBeNonnegative}
                flightState  (1,1) flightPkg.FlightState = flightPkg.FlightState.POWERED_ASCENT
                useGravity   (1,1) logical = true
                log analysisPkg.Log {mustBeScalarOrEmpty} = analysisPkg.Log.empty
            end

            [Nx, Nt] = size(state);
            xDot = zeros(Nx, Nt);

            for i = 1:Nt
                xDot(:,i) = obj.rocketModelControlWrapper(state(:,i), controlInput(:,i), atmoConditions, time(i), flightState, useGravity, log);
            end
        end

        function xDot = rocketModelControlWrapper(obj, state, controlInput, atmoConditions, time, flightState, useGravity, log)
            arguments
                obj
                state        (:,1) double
                controlInput (:,1) double
                atmoConditions (1,1) flightPkg.AtmosphericConditions
                time         (1,1) double {mustBeNonnegative}
                flightState  (1,1) flightPkg.FlightState = flightPkg.FlightState.POWERED_ASCENT
                useGravity   (1,1) logical = true
                log analysisPkg.Log {mustBeScalarOrEmpty} = analysisPkg.Log.empty
            end
            actuatorState = obj.actuatorModel.allocationMatrix * controlInput;
            xDot = obj.rocketModel(state, actuatorState, atmoConditions, time, flightState, useGravity, log);
        end

        function xDot = rocketModel(obj, state, actuatorState, atmoConditions, time, flightState, useGravity, log)
            arguments
                obj
                state         (:,1) double
                actuatorState (:,1) double
                atmoConditions (1,1) flightPkg.AtmosphericConditions
                time          (1,1) double {mustBeNonnegative}
                flightState   (1,1) flightPkg.FlightState = flightPkg.FlightState.POWERED_ASCENT
                useGravity    (1,1) logical = true
                log analysisPkg.Log {mustBeScalarOrEmpty} = analysisPkg.Log.empty
            end

            [fB, mB] = obj.getForcesMoments(state, actuatorState, atmoConditions, time, flightState, useGravity, log);
            xDot = obj.flatEarthEOM(state, time, fB, mB);
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, atmoConditions, time, flightState, useGravity, log)
            arguments
                obj
                state         (:,1) double
                actuatorState (:,1) double
                atmoConditions (1,1) flightPkg.AtmosphericConditions
                time          (1,1) double {mustBeNonnegative}
                flightState   (1,1) flightPkg.FlightState
                useGravity    (1,1) logical = true
                log analysisPkg.Log {mustBeScalarOrEmpty} = analysisPkg.Log.empty
            end

            import flightPkg.FlightState

            % Extract common state variables
            mass      = state(obj.idx.mass);
            rocketQuat = state(obj.idx.quat);
            cBI        = Utils.dirCosine(rocketQuat);
            [currentCg, ~] = obj.getCgMoi(mass);

            % Assemble forces and moments based on flight state

            switch(flightState)
                case FlightState.PRELAUNCH
                    fB = [0; 0; 0];
                    mB = [0; 0; 0];

                case FlightState.LAUNCH_RAIL
                    fBGravity = obj.computeGravity(cBI, useGravity, mass);
                    [fBAero, ~] = obj.computeAero(state, currentCg, atmoConditions);
                    [fBActuator, ~] = obj.computeActuator(state, actuatorState, currentCg, atmoConditions, time);

                    if isempty(obj.actuatorModel.motor)
                        % If no motor harnessed by actuator, get passive
                        % thrust
                        fBThrust = [obj.motor.getThrust(time); 0; 0];
                    else
                        % If motor harnessed by actuator, don't compute
                        % passive thrust
                        fBThrust = [0; 0; 0];
                    end

                    fTotal = fBGravity + fBThrust + fBAero + fBActuator;

                    fB = [fTotal(1); 0; 0];
                    mB = [0; 0; 0];

                case FlightState.POWERED_ASCENT
                    fBGravity     = obj.computeGravity(cBI, useGravity, mass);
                    [fBAero, mBAero]         = obj.computeAero(state, currentCg, atmoConditions);
                    [fBActuator, mBActuator] = obj.computeActuator(state, actuatorState, currentCg, atmoConditions, time);

                    if isempty(obj.actuatorModel.motor)
                        % If no motor harnessed by actuator, get passive
                        % thrust
                        fBThrust = [obj.motor.getThrust(time); 0; 0];
                    else
                        % If motor harnessed by actuator, don't compute
                        % passive thrust
                        fBThrust = [0; 0; 0];
                    end

                    fB = fBGravity + fBThrust + fBAero + fBActuator;
                    mB = mBAero + mBActuator;

                    if ~isempty(log)
                        log.add(time, 'aeroforces',      fBAero);
                        log.add(time, 'aeromoments',     mBAero);
                        log.add(time, 'thrust',          obj.motor.getThrust(time));
                        log.add(time, 'actuatorforces',  fBActuator);
                        log.add(time, 'actuatormoments', mBActuator);
                    end

                case FlightState.COASTING_ASCENT
                    fBGravity     = obj.computeGravity(cBI, useGravity, mass);
                    [fBAero, mBAero]         = obj.computeAero(state, currentCg, atmoConditions);
                    [fBActuator, mBActuator] = obj.computeActuator(state, actuatorState, currentCg, atmoConditions, time);

                    fB = fBGravity + fBAero + fBActuator;
                    mB = mBAero + mBActuator;

                    if ~isempty(log)
                        log.add(time, 'aeroforces',      fBAero);
                        log.add(time, 'aeromoments',     mBAero);
                        log.add(time, 'actuatorforces',  fBActuator);
                        log.add(time, 'actuatormoments', mBActuator);
                    end

                case FlightState.DESCENT
                    fBGravity     = obj.computeGravity(cBI, useGravity, mass);
                    fBParachute = obj.computeParachute(state, rocketQuat, atmoConditions, time);

                    fB = fBGravity + fBParachute;
                    mB = [0; 0; 0];

                    if ~isempty(log)
                        log.add(time, 'parachuteforces', fBParachute);
                    end

                case FlightState.LANDED
                    fB = [0; 0; 0];
                    mB = [0; 0; 0];

                otherwise
                    error('Unknown flight state: %s', string(flightState));
            end
        end

        function stateDot = flatEarthEOM(obj, state, time, fB, mB)
            arguments
                obj
                state (:,1) double
                time  (1,1) double {mustBeNonnegative}
                fB    (3,1) double
                mB    (3,1) double
            end
            mass  = state(obj.idx.mass);
            vel   = state(obj.idx.vel);
            quat  = state(obj.idx.quat);
            omega = state(obj.idx.omega);

            % Prevent mass from going below massDry
            mass = max(mass, obj.massDry);

            [~, Ib] = obj.getCgMoi(mass);

            cBI  = Utils.dirCosine(quat);
            cIB  = cBI.';

            massDot  = obj.motor.getMDot(time);
            posDot   = cIB * vel;
            velDot   = fB / mass - cross(omega, vel);
            quatDot  = Utils.quatDerivative(quat, omega);
            omegaDot = Ib \ (mB - cross(omega, Ib * omega));
            stateDot = [massDot; posDot; velDot; quatDot; omegaDot];
        end

        function computeBuildup(obj)
            arguments
                obj
            end
            % Main Author: David Reynolds
            obj.body.computeBuildup()
            obj.rocketFins.computeBuildup()

            % Added scale factors in order to line up with open rocket
            obj.body.scaleFactor = 1.51;
            obj.body.momentScaleFactor = 0.21;
            [~, numFins] = size(obj.rocketFins.fins);
            for ii = 1:numFins
                obj.rocketFins.fins(ii).scaleFactor = 1.97;
            end
        end

        function draw(obj)
            arguments
                obj
            end
            % Main Author: Kandarp Vadia
            % ========== ROCKET ==========
            rBody = obj.body.bodyParams;
            r = rBody.rocketDiameter / 2;
            xNose = linspace(rBody.noseLength, 0, 100);

            % Tangent ogive nose cone profile
            yProfile = obj.body.getNoseRadii(xNose);

            % Cylindrical body
            xBody = linspace(rBody.noseLength, rBody.noseLength + rBody.bodyLength, 50);
            yBody = r * ones(size(xBody));

            % Combine profile
            x = [xNose, xBody];
            y = [yProfile, yBody];

            % Revolve a 2D curve (x,y) about the x-axis to form a surface
            theta = linspace(0, 2*pi, 120);
            X = repmat(x(:), 1, 120);
            Y = repmat(y(:), 1, 120) .* cos(theta);
            Z = repmat(y(:), 1, 120) .* sin(theta);

            % Plot body
            figure; hold on;
            surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            axis equal; grid on;
            xlabel('Length (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('3D Rocket with Fins');
            camlight headlight; lighting gouraud;

            % ========== FINS ==========
            % Fin base position (at end of body)
            for ii = 1:length(obj.rocketFins.finParamsArr)
                fins = obj.rocketFins.finParamsArr(ii);
                translation = fins.translation;

                %xBase = rBody.noseLength + rBody.bodyLength - fins.rootChord;  % where root leading edge starts
                xBase    = translation(1);
                finColor = [0.2 0.5 0.8];

                % Define one fin shape in X–Y plane before rotation
                leRoot = [0, 0];
                teRoot = [fins.rootChord, 0];
                leTip  = [fins.finSpan * tand(fins.sweepDeg), fins.finSpan];
                teTip  = [leTip(1) + fins.tipChord, fins.finSpan];

                finX = [leRoot(1), teRoot(1), teTip(1), leTip(1)];
                finY = [leRoot(2), teRoot(2), teTip(2), leTip(2)];

                for jj = 1:fins.numFins
                    theta2 = 2*pi*(jj-1)/fins.numFins;
                    Rz       = [1 0 0; 0 cos(theta2) -sin(theta2); 0 sin(theta2) cos(theta2)];

                    Xf = finX + xBase;
                    Yf = r + finY;
                    Zf = zeros(size(finY));

                    finLocal = [Xf; Yf; Zf];
                    fin      = Rz * finLocal;

                    fill3(Xf, fin(2,:), fin(3,:), finColor, 'FaceAlpha', 0.9, 'EdgeColor', 'k');
                end
            end

            view(30,20);
        end
    end

    methods (Access = protected)
        function cp = copyElement(obj)
            cp = copyElement@matlab.mixin.Copyable(obj);

            cp.body           = rocketPkg.RocketDynamics.copyHandleProperty(obj.body,           "body");
            cp.rocketFins     = rocketPkg.RocketDynamics.copyHandleProperty(obj.rocketFins,     "rocketFins");
            cp.motor          = rocketPkg.RocketDynamics.copyHandleProperty(obj.motor,          "motor");
            cp.activeParachute = rocketPkg.RocketDynamics.copyHandleProperty(obj.activeParachute, "activeParachute");
            cp.actuatorModel  = rocketPkg.RocketDynamics.copyHandleProperty(obj.actuatorModel,  "actuatorModel");
            cp.aerodynamicsCFD = rocketPkg.RocketDynamics.copyHandleProperty(obj.aerodynamicsCFD, "aerodynamicsCFD");
        end
    end

    methods (Access = private)
        function fBGravity = computeGravity(obj, cBI, useGravity, mass)
            fBGravity = cBI * [0; 0; -9.81 * useGravity * mass];
        end

        function [fBAero, mBAero] = computeAero(obj, state, currentCg, atmoConditions)
            if isempty(obj.aerodynamicsCFD)
                [fBFins, mBFins] = obj.rocketFins.getForcesMoments(state, [], currentCg, atmoConditions);
                [fBBody, mBBody] = obj.body.getForcesMoments(state, atmoConditions);
                fBAero = fBFins + fBBody;
                mBAero = mBFins + mBBody;
            else
                [fBAero, mBAero] = obj.aerodynamicsCFD.getForcesMoments(state, atmoConditions, currentCg);
            end
        end

        function [fBActuator, mBActuator] = computeActuator(obj, state, actuatorState, currentCg, atmoConditions, time)
            [fBActuator, mBActuator] = obj.actuatorModel.getForcesMoments(state, actuatorState, currentCg, atmoConditions, time);
        end

        function fBParachute = computeParachute(obj, state, rocketQuat, atmoConditions, time)
            rocketAlt = state(obj.idx.z);
            rocketVel = state(obj.idx.vel);

            % Transition to next parachute if current one exists
            if ~isempty(obj.activeParachute) && obj.activeParachute.shouldDeployNext(rocketAlt)
                obj.activeParachute = obj.activeParachute.getNext();
            end

            if isempty(obj.activeParachute)
                fBParachute = [0; 0; 0];
                return;
            end

            if ~obj.activeParachute.isDeployed
                fprintf("'%s' deployed at t = %.2fs altitude %.2f m\n", obj.activeParachute.name, time, rocketAlt);
                obj.activeParachute.isDeployed = true;
            end

            fBParachute = obj.activeParachute.getForcesAndMoments(rocketVel, rocketQuat, atmoConditions);
        end
    end

    methods (Static, Access = private)
        function out = copyHandleProperty(in, propertyName)
            if isempty(in)
                out = in;
                return;
            end

            if isa(in, "matlab.mixin.Copyable")
                out = copy(in);
                return;
            end

            if ~isa(in, "handle")
                out = in;
                return;
            end

            try
                out = getArrayFromByteStream(getByteStreamFromArray(in));
            catch ME
                warning("RocketDynamics:ShallowCopyFallback", ...
                    "Could not deep copy property %s (%s). Reusing original handle. Error: %s", ...
                    propertyName, class(in), ME.message);
                out = in;
            end
        end
    end
end
