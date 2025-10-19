classdef BreakPoint < handle

    properties 
        break_param % altitude or acc velocity
        x_0 % optimal state
        u_0 % optimal control input
        A   % state matrix
        B   % control matrix
        gain % can be a single value or a struct of gains
    end

end