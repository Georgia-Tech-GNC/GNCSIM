% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef (Abstract) ActuatorModel < handle
    properties
        location %used to compute moment wrt CG
        qBA % quaternion of actuator frame wrt body frame
        actuatorState % states describing the actuator's position
    end
    methods (Abstract)
        [fB, mB] = getForcesMoments(obj, controlInput, rocket, CG)
    end

end