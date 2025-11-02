classdef BreakPoint < handle

    properties
        breakParam % altitude or acc velocity
        x0 % optimal state
        u0 % optimal control input
        A   % state matrix
        B   % control matrix
        gain % can be a single value or a struct of gains
    end

end