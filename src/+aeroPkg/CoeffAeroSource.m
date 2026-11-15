classdef CoeffAeroSource < aeroPkg.AeroSource
    % RASaero or CFD, OpenRocket could be adde

    properties
        model 

        % Location of the model's moment reference point, measured aft of
        % the nose tip [m]
        xAeroRef (1,1) double = 0
    end

    methods
        function obj = CoeffAeroSource(model, xAeroRef)
            arguments
                model (1,1) aeroPkg.AeroModel
                xAeroRef (1,1) double = 0
            end
            obj.model = model;
            obj.xAeroRef = xAeroRef;
        end

        function [fB, mB] = getForcesMoments(obj, state, currentCg, envConditions, powerOn)
            arguments
                obj
                state (:,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
                powerOn (1,1) logical = false
            end
            % Fin tabs not implemented 
            [aeroCon, dyn] = aeroPkg.buildAeroCondition(state, envConditions, zeros(0,1));
            aeroCon.powerOn = powerOn;

            result = obj.model.evaluateNominal(aeroCon);

            rAreftoCG = [-(currentCg - obj.xAeroRef); 0; 0];
            [fB, mB] = aeroPkg.getAeroResult(result, obj.model, dyn, rAreftoCG);
        end

        function computeBuildup(obj) 
            % Done for this with .mat tables
        end
    end
end