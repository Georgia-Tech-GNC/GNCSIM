classdef DatcomAnalysis
    %DATCOMANALYSIS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rocketDyn
    end
    
    methods
        function obj = DatcomAnalysis(rocketDyn)
            %DATCOMANALYSIS Construct an instance of this class
            %   Detailed explanation goes here
            obj.rocketDyn = rocketDyn;
        end

        function drawDatcom(obj)
            alphaRange = [0, 10];
            aeroElement = obj.rocketDyn.body
            obj.drawDatcomData()
        end
        
        function drawDatcomData(obj, alphaRange, aeroElement)
            alphas = linspace(alphaRange(1), alphaRange(2), 100);

            f = figure("Position", [200 200 700 700]);
            
            axCn = subplot(3, 2, 1);

            axCa = subplot(3, 2, 2);

            axCm = subplot(3, 2, [3, 4]);

            axDelCm = subplot(3, 2, [5, 6]);

        end
    end
end

