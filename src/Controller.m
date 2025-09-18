classdef (Abstract) Controller
    properties
        controllerID
        referenceStatesArr
    end
    methods (Abstract)
            optimalControl = compute_controls(obj, currentStatesVec, referenceStatesVec, t) % Returns control input
            obj = set_controllerID(obj, controllerID) % Sets the Controller ID
            goalReferenceState = computeGoals(t) % Gets the goal reference state based on the time
            
    end

    methods
        % Get controller ID
        function id = getControllerID(obj)
            id = obj.controllerID;
        end
    end

end