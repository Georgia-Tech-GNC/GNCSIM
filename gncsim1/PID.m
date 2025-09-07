% Main Author(s): Arlo L'Allier
%
% --- TODO ---
% Clamp on I
% - Error Messages for non same length gains and reference states and setters
%   being set the wrong class maybe? (Do last)
% - Quinternion to euler and back (don't forget to set the log value of
%   convertEuler in the masterfile and make sure all 4 quinternion values
%   are in + add note explaining that if you're going to use convertEuler
%   capability then you need to put in all 4 quinternions and in order s,x,y,z. at the end
%   anything that is not a 0 will be taken out so that the control output
%   is still just the needed pieces. (ask patrick if this last piece is
%   needed)

% note to self -- if convert error to euler, then output will only be 3
% values. Should we then convert the output back to quinternion? with
% integral and derivitive will the "0" q's (s, y?) still be 0? If not,
% things get complex. Maybe better to just have a larger value for Kp?
classdef PID < handle
    % class for PID Control
    % {detailed explanation goes here}

    properties
        controllerID = 'PID';

        KpVec;
        KiVec;
        KdVec;
        referenceStatesArr;
        convertEuler;

        int = 0;
        pastError;
        pastTime;
    end

    methods
        function obj = PID()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        % Setters ---------------------------------------------------------
        % set control ID
        function obj = set_controllerID(obj, controllerID)
            obj.controllerID = controllerID;
        end

        % set P gains
        function obj = set_KpVec(obj, KpVec)
            obj.KpVec = KpVec;
        end

        % set I gains
        function obj = set_KiVec(obj, KiVec)
            obj.KiVec = KiVec;
        end

        % set D gains
        function obj = set_KdVec(obj, KdVec)
            obj.KdVec = KdVec;
        end

        % set referenceStatesArr
        function obj = set_referenceStatesArr(obj, referenceStatesArr)
            obj.referenceStatesArr = referenceStatesArr;
        end

        % set convertEuler (logical)
        function obj = set_convertEuler(obj, log)
            obj.convertEuler = log;
        end

        % set int value
        function obj = set_int(obj, int)
            obj.int = int;
        end

        % set pastError
        function obj = set_pastError(obj, pastError)
            obj.pastError = pastError;
        end

        % set pastTime
        function obj = set_pastTime(obj, pastTime)
            obj.pastTime = pastTime;
        end
        %------------------------------------------------------------------
        
        % Getters ---------------------------------------------------------
        % get controllerID
        function controllerID = get_controllerID(obj)
            controllerID = obj.controllerID;
        end

        % get P gains
        function KpVec = get_KpVec(obj)
            KpVec = obj.KpVec;
        end

        % get I gains
        function KiVec = get_KiVec(obj)
            KiVec = obj.KiVec;
        end

        % get D gains
        function KdVec = get_KdVec(obj)
            KdVec = obj.KdVec;
        end

        % get referenceStatesArr
        function referenceStatesArr = get_referenceStatesArr(obj)
            referenceStatesArr = obj.referenceStatesArr;
        end

        % get convertEuler (logical)
        function convertEuler = get_convertEuler(obj)
            convertEuler = obj.convertEuler;
        end

        % get int value
        function int = get_int(obj)
            int = obj.int;
        end

        % get pastError value
        function pastError = get_pastError(obj)
            pastError = obj.pastError;
        end

        % get pastTime value
        function pastTime = get_pastTime(obj)
            pastTime = obj.pastTime;
        end
        %------------------------------------------------------------------

        % find goal state vector based on time
        function referenceStatesVec = computeGoals(t)
            for i = 1:height(obj.referenceStatesArr)
                if t >= obj.referenceStatesArr(i, 1)
                    referenceStatesVec = obj.referenceStatesArr(i, 2:end);
                else
                    % do nothing
                end
            end
        end

        % PID Calcs to give Control Vector Input
        function [controlVec]  = compute_controls(obj, currentStatesVec, referenceStatesVec, t)
            
            if obj.convertEuler
                % converts Quinternion inputs to Euler angles
                qs = referenceStatesVec(1);
                qx = referenceStatesVec(2);
                qy = referenceStatesVec(3);
                qz = referenceStatesVec(4);

                C11 = (qs * qs) + (qx * qx) - (qy * qy) - (qz * qz);
                C12 = 2.0 * ((qx * qy) + (qz * qs));
                C13 = 2.0 * ((qx * qz) - (qy * qs));
                C21 = 2.0 * 
                C22 = 
                C23 = 
                C31 = 
                C32 = 
                C33 = 

            end

            % P code
            error = referenceStatesVec - currectStatesVec;
            PcontrolVec = obj.KpVec .* error;

            % I code
            error = referenceStatesVec - currentStatesVec;
            obj.int = obj.int + error;
            IcontrolVec = obj.KiVec .* obj.int;

            % D code
            if obj.pastTime ~= 0
              errorPast = obj.pastError;
              error = referenceStatesVec - currentStatesVec;
              obj.pastError = error;
              dedt = (error - errorPast) / (t - obj.pastTime);
              obj.pastTime = t;
              DcontrolVec = obj.KdVec * dedt;
            end

            % calculate final control vector by adding P, I, and D control
            % vectors together.
            %% update so that DcontrolVec isn't added when is doesn't exist

            if  isempty(DcontrolVec)
                controlVec = PcontrolVec + IcontrolVec;
            else
                controlVec = PcontrolVec + IcontrolVec + DcontrolVec;
            end

        end
    end

end
