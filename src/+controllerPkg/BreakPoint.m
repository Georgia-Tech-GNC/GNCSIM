% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)
%               David Reynolds (dreynolds45@gatech.edu)

classdef BreakPoint < handle

    properties
        breakParam		% altitude or acc velocity
		
		zStar
		xStar
		uStar

		A14
		B14
        xDot14
        xDot3
        xDot2
		
		A3
		B3
		Q3 = 1 * diag([1 1 1])		%  (only used for LQR)
		R3 = 1                  %  (only used for LQR)
		K3                      %  (only used for LQR)
        
		A2  
		B2
		Q2 = diag([1 1])        %  (only used for LQR)
		R2 = 1                  %  (only used for LQR)
		K2                      %  (only used for LQR)
    end

    methods
        function obj = BreakPoint(breakParam)
            obj.breakParam = breakParam;
        end

        function computeGain(obj)
            obj.K3 = lqr(obj.A3, obj.B3, obj.Q3, obj.R3);
            obj.K2 = lqr(obj.A2, obj.B2, obj.Q2, obj.R2);
        end
    end

end