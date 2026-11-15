classdef RasaeroAeroModel < aeroPkg.AeroModel & matlab.mixin.Copyable
    % CF in body, CM_aeroRef about nose tip.

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
        cnInterp
        caOnInterp
        caOffInterp
        xCpInterp
        hasWarnedAlpha = false
    end

    methods
        function obj = RasaeroAeroModel(matFile)
            arguments
                matFile (1,1) string
            end
            S = load(matFile);
            in2m = 0.0254;

            obj.Sref = S.refArea_in2 * in2m^2;
            obj.Lref = S.refLength_in * in2m;
            obj.tabRef = struct('S', 0, 'c', 0);

            grid = {S.machGrid(:), S.alphaGrid_deg(:)};
            % 'nearest' holds the edge values outside the table
            obj.cnInterp = griddedInterpolant(grid, S.CN, "linear", "nearest");
            obj.caOnInterp = griddedInterpolant(grid, S.CA_on, "linear", "nearest");
            obj.caOffInterp = griddedInterpolant(grid, S.CA_off, "linear", "nearest");
            obj.xCpInterp = griddedInterpolant(grid, S.Xcp_in * in2m, "linear", "nearest");

            obj.machRange = [min(S.machGrid), max(S.machGrid)];
            obj.alphaRangeDeg = [min(S.alphaGrid_deg), max(S.alphaGrid_deg)];
        end

        function result = evaluateNominal(obj, aeroCon)
            M = aeroCon.M;
            alphaTot = aeroCon.alphaTot;
            phiAir = aeroCon.phiAir;

            if alphaTot > obj.alphaRangeDeg(2) && ~obj.hasWarnedAlpha
                warning("RasaeroAeroModel:alphaRange", ...
                    "alphaTot %.1f deg is past the table (%.0f deg), holding edge values.", ...
                    alphaTot, obj.alphaRangeDeg(2));
                obj.hasWarnedAlpha = true;
            end

            powerOn = isfield(aeroCon, "powerOn") && aeroCon.powerOn;
            if powerOn
                CA = obj.caOnInterp(M, alphaTot);
            else
                CA = obj.caOffInterp(M, alphaTot);
            end
            CN = obj.cnInterp(M, alphaTot);
            xCp = obj.xCpInterp(M, alphaTot);

            % Normal force
            CF = [-CA; -CN * cosd(phiAir); -CN * sind(phiAir)];

            CM = cross([-xCp / obj.Lref; 0; 0], CF);

            result.CF = CF;
            result.CM_aeroRef = CM;
            result.Ch = zeros(0, 1);
        end
    end
end