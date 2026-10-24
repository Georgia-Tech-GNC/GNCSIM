% Main Authors: Patrick Barry, Conor Foley

% Meant for implementing CFD models of rockets
% Use DATCOM if you don't have one

classdef AerodynamicsCFD < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        idx = StateIndices
    end

    properties (Access = private)
        aeroModel struct
    end

    properties
        sHat (3,:) double
        deltaSign (1,1) double
        totalLength (1,1) double
        xRefFromAft (1,1) double
    end

    methods
        function result = aeroCoeffs(raw, aeroCon)
        
            CD = 2 * raw.clean.cd;
            CN = 2 * (raw.clean.cl - raw.zero.cl);
            CmPitch = 2 * (raw.clean.cpitch - raw.zero.cpitch);
            CmRoll = 2 * (raw.clean.croll - raw.zero.croll);
         
            phiAir = aeroCon.phiAir;
            crossflowDir = [0; cosd(phiAir); sind(phiAir)];
         
            CF = -CD * [1;0;0] - CN0 * crossflowDir;
            CM = CmRoll * [1;0;0] + CmPitch * cross(xHat, crossflowDir);
         
            result.CF = CF;
            result.CM_aeroRef = CM;

        end
        
        function [fB, mB] = getAeroForcesMoments(result, q, ref, xCg)
            
            fB = q*ref.Sref*CF;
            mRef = q*ref.sRef*ref.Lref*CM;

            rRefToCG = [xCG - ref.xRef; 0; 0];
            mB = mRef + cross(rRefToCG, fB);

        end
    end
end



