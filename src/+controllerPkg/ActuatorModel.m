classdef (Abstract) ActuatorModel < handle
    properties
        location %used to compute moment wrt CG
        qBA % quaternion of actuator frame wrt body frame
    end
    methods (Abstract)
        [fB, mB] = getForcesMoments(obj, controlInput, rocket, CG)
    end

end