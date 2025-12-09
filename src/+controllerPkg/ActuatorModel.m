% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef (Abstract) ActuatorModel < handle
    properties
        location % used to compute moment wrt CG
        actuatorState % quaternion of actuator frame wrt body frame and omega
    end
    methods (Abstract)
        [fB, mB] = getForcesMoments(obj, controlInput, rocket, CG)
        [actuatorStateDot] = actuatorDynamics(obj, actuatorState)
    end

end