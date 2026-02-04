% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef (Abstract) ActuatorModel < handle

    properties (Abstract)
        allocationMatrix % maps controls to deflections
    end

    methods (Abstract)
        [fB, mB] = getForcesMoments(obj, controlInput, rocket, CG)
        [actuatorStateDot] = actuatorDynamics(obj, actuatorState)
    end
end