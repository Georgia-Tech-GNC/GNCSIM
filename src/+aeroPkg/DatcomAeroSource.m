classdef DatcomAeroSource < aeroPkg.AeroSource

    properties
        body rocketPkg.RocketBody
        rocketFins rocketPkg.RocketFins

        % Added scale factors in order to line up with open rocket
        bodyScaleFactor (1,1) double = 1.51
        bodyMomentScaleFactor (1,1) double = 0.21
        finScaleFactor (1,1) double = 1.97
    end

    methods
        function obj = DatcomAeroSource(body, rocketFins)
            arguments
                body rocketPkg.RocketBody
                rocketFins rocketPkg.RocketFins
            end
            obj.body = body;
            obj.rocketFins = rocketFins;
        end

        function [fB, mB] = getForcesMoments(obj, state, currentCg, envConditions, powerOn) %#ok<INUSD>
            arguments
                obj
                state (:,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
                powerOn (1,1) logical = false
            end
            % DATCOM has no power-on/off split, so powerOn is ignored
            [fBFins, mBFins] = obj.rocketFins.getForcesMoments(state, [], currentCg, envConditions);
            [fBBody, mBBody] = obj.body.getForcesMoments(state, envConditions);
            fB = fBFins + fBBody;
            mB = mBFins + mBBody;
        end

        function computeBuildup(obj)
            obj.body.computeBuildup()
            obj.rocketFins.computeBuildup()

            obj.body.scaleFactor = obj.bodyScaleFactor;
            obj.body.momentScaleFactor = obj.bodyMomentScaleFactor;
            for ii = 1:numel(obj.rocketFins.fins)
                obj.rocketFins.fins(ii).scaleFactor = obj.finScaleFactor;
            end
        end

        function relink(obj, body, rocketFins)
            % Keep using the same body/fins handles as RocketDynamics so
            % Monte Carlo changes to rocketDynamics.body.* still apply
            obj.body = body;
            obj.rocketFins = rocketFins;
        end
    end
end