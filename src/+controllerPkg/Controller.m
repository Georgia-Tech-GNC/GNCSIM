% Main Authors: Aditya Swami (aswami8@gatech.edu)

classdef (Abstract) Controller < handle
    properties 
        actuator
    end

    methods (Abstract)
        [fB, mB] = getForcesMoments(state, disturbances, motorParams, CG, breakPoints, time);
        gain = computeGain()
    end
end
