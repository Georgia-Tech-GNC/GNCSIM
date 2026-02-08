classdef RollRatePID < controllerPkg.Controller
    properties
        % Required by Flight scheduling logic even if unused
        breakPoints
        useActuatorModel = true
        referenceAttitudes = []

        % Gains / timing
        Kp
        Ki
        Kd
        dt

        % Limits / anti-windup
        tauMax
        intMin
        intMax

        % State
        intErr = 0
        prevErr = 0
        prevTime = 0
        wDes = 0

        % Optional filter object (external filtering is cleaner)
        lp = []
        idx = StateIndices
    end

    methods
        function obj = RollRatePID(Kp, Ki, Kd, dt, tauMax, intMin, intMax, lp)
            obj.Kp = Kp; obj.Ki = Ki; obj.Kd = Kd;
            obj.dt = dt; obj.tauMax = tauMax;
            obj.intMin = intMin; obj.intMax = intMax;

            if nargin >= 8
                obj.lp = lp;
            end

            % Never switch schedules
            obj.breakPoints = struct('cumVelocityInterval', obj.dt);
        end

        % Seems to be defined, I think I might just need to fill in the
        % blanks
        function u = getControl(obj, state, ~, ~)
            wx = state(obj.idx.omega_x); % roll rate (rad/s)

            if ~isempty(obj.lp)
                wx = obj.lp.update(wx);
            end

            wxRef = 0;
            err = wxRef - wx;

            obj.intErr = obj.intErr + err * obj.dt;
            obj.intErr = min(max(obj.intErr, obj.intMin), obj.intMax);

            derr = (err - obj.prevErr) / obj.dt;
            obj.prevErr = err;

            tau_unsat = obj.Kp*err + obj.Ki*obj.intErr + obj.Kd*derr;
            tau = min(max(tau_unsat, -obj.tauMax), obj.tauMax);

            % Output as "control input" vector consumed by actuator model
            u = [tau; 0; 0];
        end

        function u = getControlS(obj, state, currentCumVelocity, currentTime)
            wx = state(obj.idx.omega_x); % roll rate (rad/s)

            if ~isempty(obj.lp)
                wx = obj.lp.update(wx);
            end
            
            error = wx - obj.wDes;
            newIntTerm = obj.intErr + error * obj.dt;
            

            p = obj.Kp * error;
            i = obj.Ki * newIntTerm * obj.dt;
            d = (error - obj.prevErr) / obj.dt; % May also be current time - previous time
            tau_unsat = p + i + d;
      
            obj.breakPoints = struct('cumVelocityInterval', obj.dt);
            

            obj.intErr = min(max(obj.intErr, obj.intMin), obj.intMax);
            obj.prevErr = error;
            obj.previousTime = currentTime;
            
            satHigh = (tau_unsat > obj.tauMax) && (error > 0);
            satLow = (tau_unsat < -obj.tauMax) && (error < 0);

            if(~satHigh && ~satLow) 
                obj.intErr = newIntTerm;
            end
            
            u = tau_command;
        end

        function populateBreakPoints(~)
            % No gain scheduling
        end
    end
end