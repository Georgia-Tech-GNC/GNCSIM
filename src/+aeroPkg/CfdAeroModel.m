classdef CfdAeroModel < aeroPkg.AeroModel & matlab.mixin.Copyable
    % CFD grid

    properties (SetAccess = protected)
        Sref
        Lref
        tabRef
    end

    properties (SetAccess = protected)
        machRange (1,2) double
        alphaRangeDeg (1,2) double
    end

    properties (Access = private)
        caInterp
        cnInterp
        cmpInterp
        hasWarnedAlpha (1,1) logical = false
        hasWarnedMach (1,1) logical = false
    end

    methods
        function obj = CfdAeroModel(matFile)
            arguments
                matFile (1,1) string
            end
            S = load(matFile);

            obj.Sref = S.refArea_m2;
            obj.Lref = S.refLength_m;
            obj.tabRef = struct('S', 0, 'c', 0);

            grid = {S.machGrid(:), S.alphaGrid_deg(:)};
            obj.caInterp = griddedInterpolant(grid, S.CA, "linear", "nearest");
            obj.cnInterp = griddedInterpolant(grid, S.CN, "linear", "nearest");
            obj.cmpInterp = griddedInterpolant(grid, S.CMP, "linear", "nearest");

            obj.machRange = [min(S.machGrid), max(S.machGrid)];
            obj.alphaRangeDeg = [min(S.alphaGrid_deg), max(S.alphaGrid_deg)];
        end

        function result = evaluateNominal(obj, aeroCon)
            M = aeroCon.M;
            alphaTot = aeroCon.alphaTot;
            phiAir = aeroCon.phiAir;
            obj.warnOutOfRange(M, alphaTot);

            CA = obj.caInterp(M, alphaTot);
            CN = obj.cnInterp(M, alphaTot);
            CMP = obj.cmpInterp(M, alphaTot);

            % Normal force 
            CF = [-CA; -CN * cosd(phiAir); -CN * sind(phiAir)];

            CM = -CMP * [0; -sind(phiAir); cosd(phiAir)];

            result.CF = CF;
            result.CM_aeroRef = CM;
            result.Ch = zeros(0, 1);
        end

    end

    methods (Access = private)
        function warnOutOfRange(obj, M, alphaTot)
            if M > obj.machRange(2) && ~obj.hasWarnedMach
                warning("CfdAeroModel:machRange", ...
                    "Mach %.3f is past the CFD grid (%.3f), holding edge values.", M, obj.machRange(2));
                obj.hasWarnedMach = true;
            end
            if alphaTot > obj.alphaRangeDeg(2) && ~obj.hasWarnedAlpha
                warning("CfdAeroModel:alphaRange", ...
                    "alphaTot %.1f deg is past the CFD grid (%.0f deg), holding edge values.", ...
                    alphaTot, obj.alphaRangeDeg(2));
                obj.hasWarnedAlpha = true;
            end
        end
    end
end