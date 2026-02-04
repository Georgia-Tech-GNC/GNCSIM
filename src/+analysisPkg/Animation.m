% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef Animation
	properties
		log
		rocketDyn
	end
	
	methods
		function obj = Animation(log, rocketDyn)
			obj.log = log;
			obj.rocketDyn = rocketDyn;
		end

		function run(obj)
			obj.initFig()
			obj.drawStatic()
			obj.drawTraj()
			[hT, tT] = obj.drawRocket();
			obj.animate(hT, tT);
		end

		function f = initFig(obj)
			screenSize = get(0, 'ScreenSize'); % [left, bottom, width, height]
    		figWidth = 1600;
    		figHeight = 900;
    		xPos = (screenSize(3) - figWidth) / 2;
    		yPos = (screenSize(4) - figHeight) / 2;
		
    		f = figure('Color', 'w');
    		set(f, 'Position', [xPos, yPos, figWidth, figHeight]);

			axis equal tight vis3d off
			camproj('perspective');
			camva(40);
			hold on
			rotate3d on % rotation pre selcted
		end

		function drawStatic(obj)
			% Draw Grid
			gridLength = 400;
			numLines = 11;
			originLength = 3;
			
			% Draw Grid
			for ii=0:numLines
				xval = ((-numLines / 2) + ii) * (gridLength / numLines);
				X = [xval, xval];
				Y = [-gridLength/2, gridLength/2];
				Z = [0, 0];
				plot3(X,Y,Z, 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
				plot3(Y,X,Z, 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
			end

			% Draw Orgin
			plot3([0, originLength], [0, 0], [0, 0], 'LineWidth', 2, 'Color', [1 0 0]);
			plot3([0, 0], [0, originLength], [0, 0], 'LineWidth', 2, 'Color', [0 1 0]);
			plot3([0, 0], [0, 0], [0, originLength], 'LineWidth', 2, 'Color', [0 0 1]);
		end
		
		function [hT, hThrust] = drawRocket(obj)
			hT = hgtransform('Parent', gca);

            % ========== ROCKET ==========
            rBody = obj.rocketDyn.body.bodyParams;
            r = rBody.rocketDiameter/2;
            xNose = linspace(rBody.noseLength, 0, 100);

            % Tangent ogive nose cone profile
            yProfile = obj.rocketDyn.body.getNoseRadii(xNose);

            % Cylindrical body
            xBody = linspace(rBody.noseLength, rBody.noseLength + rBody.bodyLength, 50);
            yBody = r * ones(size(xBody));

            % Combine profile
            x = [xNose, xBody];
            y = [yProfile, yBody];

            % Revolve a 2D curve (x,y) about the x-axis to form a surface
            theta = linspace(0, 2*pi, 120);
            X = repmat(x(:), 1, 120);
            Y = repmat(y(:), 1, 120).*cos(theta);
            Z = repmat(y(:), 1, 120).*sin(theta);
			
			surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'Parent', hT);
			camlight headlight; lighting gouraud;

            % ========== FINS ==========
            % Fin base position (at end of body)
            for ii=1:length(obj.rocketDyn.rocketFins.finParamsArr)
                fins = obj.rocketDyn.rocketFins.finParamsArr(ii);
                translation = fins.translation;

                %xBase = rBody.noseLength + rBody.bodyLength - fins.rootChord;  % where root leading edge starts
                xBase = translation(1);
                finColor = [0.2 0.5 0.8];
    
                % Define one fin shape in X–Y plane before rotation
                leRoot = [0, 0];
                teRoot = [fins.rootChord, 0];
                leTip  = [fins.finSpan * tand(fins.sweepDeg), fins.finSpan];
                teTip  = [leTip(1) + fins.tipChord, fins.finSpan];
    
                finX = [leRoot(1), teRoot(1), teTip(1), leTip(1)];
                finY = [leRoot(2), teRoot(2), teTip(2), leTip(2)];

                % For each fin, rotate and position around body
                for jj = 1:fins.numFins
                    theta2 = 2*pi*(jj-1)/fins.numFins;  % angular position
                    % Rotation matrix around X-axis (rocket axis)
                    Rz = [1 0 0; 0 cos(theta2) -sin(theta2); 0 sin(theta2) cos(theta2)];
    
                    % Shift fin so its root attaches to rocket body
                    Xf = finX + xBase;
                    Yf = r + finY;
                    Zf = zeros(size(finY));
    
                    % Stack to rotate (not necessary here because we rotate by construction)
                    finLocal = [Xf; Yf; Zf];
                    fin = Rz * finLocal;
    
                    % Draw fin as patch
                    fill3(Xf, fin(2,:), fin(3,:), finColor, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'Parent', hT);
                end
			end

			xBase = rBody.noseLength + rBody.bodyLength;
    			
			% Initialize an empty line. We use [xBase, xBase] so it starts at the tail.
			% We'll update the second X point and Y/Z points in the animation loop.
			hThrust = line([xBase, xBase], [0, 0], [0, 0], ...
    			'Color', [1, 0.5, 0], ... % Orange thrust
    			'LineWidth', 2, ...
    			'Parent', hT);

		end

		function drawTraj(obj)
			x_pos = obj.log.getState('x');
            y_pos = obj.log.getState('y');
            z_pos = obj.log.getState('z');
			vx = obj.log.getState('vx');
            vy = obj.log.getState('vy');
            vz = obj.log.getState('vz');
			q0 = obj.log.getState('q0');
            q1 = obj.log.getState('q1');
            q2 = obj.log.getState('q2');
            q3 = obj.log.getState('q3');
            time = obj.log.time(1:obj.log.currentIdx);

			aniState = [x_pos; y_pos; z_pos; vx; vy; vz; q0; q1; q2; q3];
			aniState = aniState(:, 1:obj.log.currentIdx);

			%plot3(x_pos, y_pos, z_pos, 'LineWidth', 2, 'Color', [0.3, 0.95, 0.95], 'AlignVertexCenters', 'on');
			
			totalSimTime = obj.log.time(end);
			hoopsTime = 2:2:totalSimTime;
			hoopSize = 50;

			for t=hoopsTime
				[~, tIndex] = min(abs(time - t));
				pos = aniState(1:3, tIndex);
				vel = aniState(4:6, tIndex);
				
				quat = aniState(7:10, tIndex); 
				R = Utils.dirCosine(quat).';
				velI = R * vel;

				obj.plotCircle3D(gca, pos, velI, hoopSize, [0.5, 0.5, 0.5])
			end
		end
		
		function animate(obj, rocketTransform, thrustTransform)
            saveVideo = true;
            filename = 'animation.mp4';
			rBody = obj.rocketDyn.body.bodyParams;
			thrustScale = 4 / max(obj.log.thrust);
			
			q0 = obj.log.getState('q0');
            q1 = obj.log.getState('q1');
            q2 = obj.log.getState('q2');
            q3 = obj.log.getState('q3');
			x_pos = obj.log.getState('x');
            y_pos = obj.log.getState('y');
            z_pos = obj.log.getState('z');
            vx = obj.log.getState('vx');
            vy = obj.log.getState('vy');
            vz = obj.log.getState('vz');
            time = obj.log.time(1:obj.log.currentIdx);

			aniState = [q0; q1; q2; q3; x_pos; y_pos; z_pos; vx; vy; vz];
			aniState = aniState(:, 1:obj.log.currentIdx);

			txtHandle = annotation('textbox', [0.05, 0.8, 0.3, 0.15], ...
        		'String', 'Initializing...', ...
        		'Units', 'normalized', ...
        		'FontUnits', 'normalized', ... % Scales with resolution
        		'FontSize', 0.03, ...          % 3% of figure height
        		'Color', 'black', ...
				'Interpreter', 'tex');
        		%'BackgroundColor', [0, 0, 0, 0.6], ... % Translucent black
        		%'EdgeColor', 'green', ...
        		%'FontName', 'Consolas', ...
        		

			totalSimTime = obj.log.time(end);
			camupVector = [0; 0; 1];
			camOffset = [4, -4, 8];

			flipX = diag([-1, -1, 1]);

            if saveVideo
                fps = 60; % Frames per second for the output file
                v = VideoWriter(filename, 'MPEG-4');
                v.FrameRate = fps;
                v.Quality = 95;
                open(v);
                dt = 1/fps; % Fixed time step for video
            else
                dt = 0;
                tic;
            end

			simTime = 0.0;
			
			camup('manual');
			while simTime < totalSimTime
				if saveVideo
                    simTime = simTime + dt;
                else
                    simTime = toc;
                end

                if simTime > totalSimTime, break; end
				
				stateInterp = interp1(time, aniState.' ,simTime);
				currentThrust = interp1(time, obj.log.thrust, simTime);

				quat = stateInterp(1:4);       % [q0, q1, q2, q3]
                vel = stateInterp(8:10);
                speed = norm(vel);
				%quat_rotated = quat * quat_y90;
				
				pos = stateInterp(5:7);     % [x, y, z]
				R = Utils.dirCosine(quat).' * flipX;

				T = eye(4);
        		T(1:3, 1:3) = R;            % Insert Rotation
        		T(1:3, 4) = pos';           % Insert Translation

				cameraPoint = pos + camOffset;
        		
        		set(rocketTransform, 'Matrix', T);

				xBase = rBody.noseLength + rBody.bodyLength;
    			newXData = [xBase, xBase + currentThrust * thrustScale];
    			
    			% Update the handle
    			set(thrustTransform, 'XData', newXData);

				camtarget(pos);
				campos(cameraPoint);
				camup(camupVector);
				
				% Update Text
				txtHandle.String = { ...
            	sprintf('Time: %.2f s', simTime), ...
            	sprintf('Altitude: %.1f m', pos(3)), ...
            	sprintf('Speed: %.2f m/s', speed)};

        		drawnow;
                if saveVideo
                    frame = getframe(gcf);
                    writeVideo(v, frame);
                end
            end
            if saveVideo
                close(v);
                fprintf('Video saved as %s\n', filename);
            end
		end
	end

	methods (Static)
		function cir = plotCircle3D(ax, center, normal, radius, color)
			% Funciton done by Christian Reinbacher, slight modifcations done by David Reynolds
			% https://www.mathworks.com/matlabcentral/fileexchange/26588-plot-circle-in-3d
			theta=0:0.02:2*pi;
			v=null(normal');
			points=repmat(center, 1, size(theta,2)) + radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
			cir = plot3(ax, points(1,:), points(2,:), points(3,:), 'Color', color);
		end
	end
end
