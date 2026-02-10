% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef DatcomAnalysis
    %DATCOMANALYSIS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rocketDyn rocketPkg.RocketDynamics
		N (1,1) double = 100
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
            arguments
                obj
                alphaRange (1,2) double
                aeroElement rocketPkg.DatcomManager
            end
            alphas = linspace(alphaRange(1), alphaRange(2), obj.N);
            alt = 0;
            altArr = repmat(alt, 1, obj.N);
            mach = 0.2;
            machArr = repmat(mach, 1, obj.N);

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

        function drawForcesAndMoments(obj, alphaRange)
            arguments
                obj
                alphaRange (1,2) double
            end
			nominalMass = 20;
			nominalPos = [0; 0; 0];
			nominalquat = Utils.eulerToQuat([0, 0, 0]);
			nominalOmega = [0; 0; 0];
			alphas = linspace(alphaRange(1), alphaRange(2), obj.N);
			nominalSpeed = 50;
			
			fBAeroFin = zeros(3, obj.N);
			mBAeroFin = zeros(3, obj.N);
			fBAeroBody = zeros(3, obj.N);
			mBAeroBody = zeros(3, obj.N);
			fBAero = zeros(3, obj.N);
			mBAero = zeros(3, obj.N);

			for ii=1:obj.N
				alpha = alphas(ii);
				vel = Utils.dirCosine(Utils.eulerToQuat([0, alpha, 0])) * [nominalSpeed; 0; 0];
				
				state = [nominalMass; nominalPos; vel; nominalquat; nominalOmega];
				disturbances = [];
				xcg = obj.rocketDyn.cgWet;
			
				[fBAeroFin(:, ii), mBAeroFin(:, ii)] = obj.rocketDyn.rocketFins.getForcesMoments(state, xcg, disturbances);
            	[fBAeroBody(:, ii), mBAeroBody(:, ii)] = obj.rocketDyn.body.getForcesMoments(state, disturbances);
            	fBAero(:, ii) = fBAeroFin(:, ii) + fBAeroBody(:, ii);
            	mBAero(:, ii) = mBAeroFin(:, ii) + mBAeroBody(:, ii);
			end
			
			f = figure("Position", [200 200 1200 500], "Name", "Pitching Moment for Rocket");

			% --- Total Moment (Body + Fins) ---
			subplot(2,3,1);
			Mz_total = mBAero(2, :);
			plot(alphas, Mz_total)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('M_y Total')
			
			subplot(2,3,4);
			dMz_total = gradient(Mz_total, alphas);
			plot(alphas, dMz_total)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('dM_y/d\alpha Total')
			
			
			% --- Body-Only Moment ---
			subplot(2,3,2);
			Mz_body = mBAeroBody(2, :);
			plot(alphas, Mz_body)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('M_y Body')
			
			subplot(2,3,5);
			dMz_body = gradient(Mz_body, alphas);
			plot(alphas, dMz_body)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('dM_y/d\alpha Body')
			
			
			% --- Fin-Only Moment ---
			subplot(2,3,3);
			Mz_fin = mBAeroFin(2, :);
			plot(alphas, Mz_fin)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('M_y Fins')
			
			subplot(2,3,6);
			dMz_fin = gradient(Mz_fin, alphas);
			plot(alphas, dMz_fin)
			grid on;
			xlabel('Angle of Attack (deg)')
			ylabel('dM_y/d\alpha Fins')
		end
    end
end

