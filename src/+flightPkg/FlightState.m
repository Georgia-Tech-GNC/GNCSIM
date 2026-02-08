classdef FlightState < uint8
    enumeration
        PRELAUNCH       (1)
        LAUNCH_RAIL     (2)
        POWERED_ASCENT  (3)
        COASTING_ASCENT (4)
        DESCENT         (5)
        LANDED          (6)
    end
end