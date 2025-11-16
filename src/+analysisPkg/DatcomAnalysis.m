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
            body = obj.rocketDyn.body;
            obj.drawDatcomData(alphaRange, body)

            fin = obj.rocketDyn.rocketFins.fins(1);
            obj.drawDatcomData(alphaRange, fin)
        end
        
        function drawDatcomData(obj, alphaRange, aeroElement)
            alphas = linspace(alphaRange(1), alphaRange(2), 100);
            alt = 0;
            altArr = repmat(alt, 1, 100);
            mach = 0.2;
            machArr = repmat(mach, 1, 100);

            f = figure("Position", [200 200 700 700]);
            
            axCn = subplot(3, 2, 1);

            cnArr = aeroElement.cnInterp(alphas, machArr, altArr);
            plot(alphas, cnArr)
            grid on;
            xlabel('Angle of Attack (deg)')
            ylabel('Normal Coefficient C_n')

            axCa = subplot(3, 2, 2);
            caArr = aeroElement.caInterp(alphas, machArr, altArr);
            plot(alphas, caArr)
            grid on;
            xlabel('Angle of Attack (deg)')
            ylabel('Axial Coefficient C_a')

            axCm = subplot(3, 2, [3, 4]);
            cmArr = aeroElement.cmInterp(alphas, machArr, altArr);
            plot(alphas, cmArr)
            grid on;
            xlabel('Angle of Attack (deg)')
            ylabel('Moment Coefficient C_m')

            axDelCm = subplot(3, 2, [5, 6]);
            dCm_dAlpha = gradient(cmArr, alphas);   % dCm/dα
            plot(alphas, dCm_dAlpha)
            grid on;
            xlabel('Angle of Attack (deg)')
            ylabel('dC_m/d\alpha')
        end

        function drawForcesAndMoments(obj)
    end
end

