% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef BreakPoint < handle

    properties
        breakParam		% altitude or acc velocity
		
		zStar
		xStar
		uStar

		A14
		B14
		
		A3
		B3
		Q3		%  (only used for LQR)
		R3
		K3

		A2
		B2
		Q2
		R2
		K2
    end

end