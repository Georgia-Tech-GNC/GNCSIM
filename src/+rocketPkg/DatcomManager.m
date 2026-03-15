% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Kandarp Vadia (kvadia3@gatech.edu)

classdef DatcomManager < handle
	% DATCOMMANAGER Base class for managing DATCOM file generation,
	% execution, and aerodynamic data import. Provides functionality for
	% creating input files, defining flight conditions, and running DATCOM.

	properties (Constant)
		idx = StateIndices
	end

	properties
		fileID				% File identifier for the currently open DATCOM input file
		flightConParams		% Structure containing flight condition parameters (Mach, altitude, alpha)
		caInterp
		cmInterp
		cnInterp
		aero
		scaleFactor = 1;
		momentScaleFactor = 1;
		aeroSourceMode = "runtime_cache";
		precomputedAeroDir = "";
        lastDatcomDeckText = "";
	end

	methods
		% Constructor
		function obj = DatcomManager(flightConParams)
			% Initializes the DatcomManager object and stores flight condition parameters
			obj.flightConParams = flightConParams;
		end

		function openDatcom(obj, datcomFilename)
			% Opens a new DATCOM input file for writing.
			% Throws an error if the file cannot be opened.
			obj.fileID = fopen(datcomFilename, 'w');
            if obj.fileID == -1
                error('Could not open file for writing.');
			end
		end

		function addFlightCondition(obj)
			% Writes the $FLTCON namelist to the DATCOM input file.
			% Includes Mach numbers, altitudes (m), and angle-of-attack schedule.
			machNumbers = obj.flightConParams.machNumbers;
            altitudes = obj.flightConParams.altitudes;
            alphasDeg   = obj.flightConParams.alphasDeg;
			
			% Write Mach numbers
			if ~isempty(machNumbers)
                fprintf(obj.fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(machNumbers), machNumbers(1));
                % Loop through the rest of the Mach numbers, printing each on a new line
                for i = 2:length(machNumbers)
                    fprintf(obj.fileID, ' %.4f,\n', machNumbers(i));
                end
            end

            % Write altitudes
            if ~isempty(altitudes)
                fprintf(obj.fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes), altitudes(1));
                for i = 2:length(altitudes)
                    fprintf(obj.fileID, ' %.4f,\n', altitudes(i));
                end
            end

			% Write angle-of-attacks
            if ~isempty(alphasDeg)
                fprintf(obj.fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphasDeg), alphasDeg(1));
                % Loop to the SECOND-TO-LAST element, printing a comma
                for i = 2:length(alphasDeg)
                    fprintf(obj.fileID, ' %.4f,\n', alphasDeg(i));
                end
			end
			% End of flight condition block
            fprintf(obj.fileID, ' LOOP=2.0$\n');
        end

		function closeDatcom(obj)
			% Closes the DATCOM input file

			% Set units to meters
			fprintf(obj.fileID, 'DIM M\n');
			fprintf(obj.fileID, 'NEXT CASE\n');
			fclose(obj.fileID);
		end

		function buildInterps(obj, aero)
            alpha = unique(aero.alpha(:));
            mach = unique(aero.mach(:));
            alt = unique(aero.alt(:));
            
            obj.caInterp = griddedInterpolant({alpha, mach, alt}, aero.ca, "linear");
            obj.cmInterp = griddedInterpolant({alpha, mach, alt}, aero.cm, "linear");
            obj.cnInterp = griddedInterpolant({alpha, mach, alt}, aero.cn, "linear");
        end

		function [ca, cm, cn] = aeroInterp(obj, mach, aoaDeg, alt)
			% TODO change this function to generate functions

            ca = obj.scaleFactor * obj.caInterp(aoaDeg, mach, alt);
            cm = obj.momentScaleFactor * obj.cmInterp(aoaDeg, mach, alt);
			cn = obj.scaleFactor * obj.cnInterp(aoaDeg, mach, alt);

			alphaRange = [min(obj.aero.alpha(:)), max(obj.aero.alpha(:))];
    		machRange  = [0, max(obj.aero.mach(:))];
    		altRange   = [min(obj.aero.alt(:)),  max(obj.aero.alt(:))];

    		% Check bounds
            if 0
    		    if aoaDeg < alphaRange(1) || aoaDeg > alphaRange(2)
        		    warning('AoA %.2f° is out of range [%.2f, %.2f].', aoaDeg, alphaRange(1), alphaRange(2));
    		    end
    		    if mach < machRange(1) || mach > machRange(2)
        		    warning('Mach %.2f is out of range [%.2f, %.2f].', mach, machRange(1), machRange(2));
    		    end
    		    if alt < altRange(1) || alt > altRange(2)
        		    warning('Altitude %.2f m is out of range [%.2f, %.2f].', alt, altRange(1), altRange(2));
                end
            end
		end

		function [fC, mC] = getForcesMomentsDatcomBody(obj, state, vC)
			% Aerodynamic coefficients
           	vA = sqrt(vC(1)^2+vC(2)^2+vC(3)^2);
           	alpha = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = vA/alpha;
           	AoAz = atan2d(vC(3), vC(1));  % main AoA
			AoAy = atan2d(vC(2), vC(1));  % AoA for y axis (very similar to sideslip)

			% compute total angle of attack
			totalAoA = acosd(cosd(AoAz) * cosd(AoAy));

			theta = atan2(vC(2), vC(3) + 1e-6);
			
			Rx = [
				1,           0,          0;
				0,           cos(theta), sin(theta);
				0,           -sin(theta), cos(theta)
			];
			
			% calculate coeffients in wind frame
			pos = state(obj.idx.pos);
           	altitude = pos(3);  % z componet of position
			[ca, cm, cn] = obj.aeroInterp(M, totalAoA, altitude);
	
           	S = obj.aero.sref;
           	c = obj.aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * vA^2 * rho;

			% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
           	fW = [-A; 0; -N];
		   	mW = [0; M; 0];
			
			% Rotate forces and moments back to componet frame
			fC = Rx * fW;
			mC = Rx * mW;
		end

		function [fC, mC] = getForcesMomentsDatcom(obj, state, vC)

			% Aerodynamic coefficients
           	vA = sqrt(vC(1)^2+vC(2)^2+vC(3)^2);
           	alpha = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = vA/alpha;
           	AoA = atan2d(vC(3),vC(1));  % main AoA

           	pos = state(obj.idx.pos);
           	altitude = pos(3);  % z componet of position
           	[ca, cm, cn] = obj.aeroInterp(M, AoA, altitude);
	
           	S = obj.aero.sref;
           	c = obj.aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * vA^2 * rho;

           	% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
            fC = [-A; 0; -N];
		   	mC = [0; M; 0];
		end

		function aero = resolveAeroData(obj, datcomFilename)
			mode = string(obj.aeroSourceMode);
			if strlength(mode) == 0
				mode = "runtime_cache";
			end

			deckText = fileread(datcomFilename);
			bytes = uint8(deckText);
			checksum = uint32(0);
			for i = 1:numel(bytes)
				checksum = checksum + uint32(bytes(i)) * uint32(mod(i, 65535) + 1);
			end
			byteCount = numel(bytes);

			thisFile = mfilename('fullpath');
			rocketPkgDir = fileparts(thisFile);
			srcDir = fileparts(rocketPkgDir);
			projectRoot = fileparts(srcDir);
			cacheFile = fullfile(projectRoot, 'DATCOM', 'cache', sprintf('aero_%08X_%d.mat', checksum, byteCount));

			precomputedDir = char(obj.precomputedAeroDir);
			if isempty(precomputedDir)
				precomputedDir = fullfile(projectRoot, 'DATCOM', 'precomputed');
			end
			precomputedFile = fullfile(precomputedDir, sprintf('aero_%08X_%d.mat', checksum, byteCount));

			switch mode
				case "precomputed_only"
					if exist(precomputedFile, 'file')
						S = load(precomputedFile, 'aero');
						aero = S.aero;
						return;
					end

					if exist(cacheFile, 'file')
						S = load(cacheFile, 'aero');
						aero = S.aero;
						return;
					end

					error("Syndrome:DATCOM:PrecomputedMissing", ...
						"Precomputed DATCOM aero not found for deck '%s'. Expected '%s' or cached '%s'.", ...
						datcomFilename, precomputedFile, cacheFile);

				case "prefer_precomputed"
					if exist(precomputedFile, 'file')
						S = load(precomputedFile, 'aero');
						aero = S.aero;
						return;
					end

					aero = rocketPkg.DatcomManager.runDatcomCached(datcomFilename);
					if ~exist(precomputedDir, 'dir')
						mkdir(precomputedDir);
					end
					save(precomputedFile, 'aero', 'deckText');

				case "runtime_cache"
					aero = rocketPkg.DatcomManager.runDatcomCached(datcomFilename);

				otherwise
					error("Syndrome:DATCOM:UnknownAeroMode", ...
						"Unknown aero source mode '%s'. Use runtime_cache, prefer_precomputed, or precomputed_only.", ...
						mode);
			end
		end
	end

	methods (Static)
		function runDatcom(datcomFilename)
			% runDatcom  Execute DATCOM using an absolute path to the executable.
			% Works even if caller cd's into a temp working directory.
			datcomPaths = rocketPkg.DatcomManager.getDatcomPaths();
			exePath = rocketPkg.DatcomManager.getDatcomExecutable(datcomPaths);

			% -------------------------
			% Prepare DATCOM input file
			% -------------------------
			if ~isfile(datcomFilename)
				error("Syndrome:DATCOM:InputDeckNotFound", ...
					"DATCOM input deck not found: %s", datcomFilename);
			end

			% DATCOM expects the input deck to be named "for005" in the working directory
			try
				copyfile(datcomFilename, fullfile(pwd, 'for005'));
			catch ME
				error("Syndrome:DATCOM:InputCopyFailed", ...
					"Failed to copy DATCOM deck '%s' into working directory '%s': %s", ...
					datcomFilename, pwd, ME.message);
			end

			% -------------------------
			% Execute DATCOM
			% -------------------------
			if ispc
				% Windows: run the .exe
				command = sprintf('echo for005 | "%s"', exePath);
			else
				% Unix/mac: ensure executable bit, then run
				system(sprintf('chmod +x "%s"', exePath));
				command = sprintf('printf ''for005\n'' | "%s"', exePath);
			end

			[status, cmdout] = system(command);
			if status ~= 0
				error("Syndrome:DATCOM:ExecutionFailed", ...
					"DATCOM execution failed using '%s' (status=%d, arch=%s, workdir=%s). %s", ...
					exePath, status, computer('arch'), pwd, rocketPkg.DatcomManager.formatSystemOutput(cmdout));
			end

			% DATCOM writes outputs in the current working directory (e.g., datcom.out).
			% Caller (runDatcomCached) is responsible for importing datcom.out from cwd.
		end

		function aero = runDatcomCached(datcomFilename)
			% Shared cache: in-memory (fast) + disk (shared across local workers).
			% Keyed by exact deck text.

			persistent memCache
			if isempty(memCache)
				memCache = containers.Map('KeyType','char','ValueType','any');
			end

			deckText = fileread(datcomFilename);
			key = deckText;

			% Fast path: in-memory
			if isKey(memCache, key)
				aero = memCache(key);
				return;
			end

			% Disk cache path (shared across workers)
			thisFile = mfilename('fullpath');
			rocketPkgDir = fileparts(thisFile);
			srcDir = fileparts(rocketPkgDir);
			projectRoot = fileparts(srcDir);
			datcomSourceDir = fullfile(srcDir, 'DATCOM');
			cacheDir = fullfile(projectRoot, 'DATCOM', 'cache');
			if ~exist(cacheDir, 'dir'), mkdir(cacheDir); end

			% Deterministic filename without Java: use a simple checksum + length.
			% (Collision risk is extremely low for your use; if you want zero-risk, we can switch to MD5.)
			bytes = uint8(deckText);
			checksum = uint32(0);
			for i = 1:numel(bytes)
				checksum = checksum + uint32(bytes(i)) * uint32(mod(i, 65535) + 1);
			end
			byteCount = numel(bytes);
			cacheFile = fullfile(cacheDir, sprintf('aero_%08X_%d.mat', checksum, byteCount));
			lockDir   = [cacheFile, '.lock'];
			lockMetaFile = fullfile(lockDir, 'owner.txt');
			staleLockSeconds = 45;
			waitTimeoutSeconds = 90;
			waitStart = tic;

			% Disk cache hit
			if exist(cacheFile, 'file')
				S = load(cacheFile, 'aero');
				aero = S.aero;
				memCache(key) = aero;
				return;
			end

			% Acquire lock (atomic mkdir is cross-platform)
			while true
				% Suppress spam when the lock directory already exists
				warnState = warning('off', 'MATLAB:MKDIR:DirectoryExists');
				ok = mkdir(lockDir);
				warning(warnState);

				if ok
					fidLock = fopen(lockMetaFile, 'w');
					if fidLock ~= -1
						fprintf(fidLock, 'pid=%d\n', feature('getpid'));
						hostName = getenv('HOSTNAME');
						if isempty(hostName)
							hostName = 'unknown';
						end
						fprintf(fidLock, 'host=%s\n', hostName);
						fprintf(fidLock, 'created=%s\n', datestr(now, 31));
						fclose(fidLock);
					end
					break;
				end

				% If another worker finished while we waited, load and return
				if exist(cacheFile, 'file')
					S = load(cacheFile, 'aero');
					aero = S.aero;
					memCache(key) = aero;
					return;
				end

				info = dir(lockDir);
				lockAgeSeconds = inf;
				if ~isempty(info)
					lockAgeSeconds = (now - info(1).datenum) * 86400;
				end
				if lockAgeSeconds > staleLockSeconds
					if exist(lockDir, 'dir')
						rmdir(lockDir, 's');
					end
					continue;
				end

				if toc(waitStart) > waitTimeoutSeconds
					error("Syndrome:DATCOM:LockTimeout", ...
						"Timed out waiting for DATCOM cache lock '%s'. If another MATLAB worker is not actively building this cache entry, delete the stale lock directory and retry.", ...
						lockDir);
				end
				pause(0.05);
			end

			% Lock holder: re-check cache after acquiring lock
			if exist(cacheFile, 'file')
				rocketPkg.DatcomManager.removeLockPath(lockDir);
				S = load(cacheFile, 'aero');
				aero = S.aero;
				memCache(key) = aero;
				return;
			end

			% Run DATCOM in a temp working directory to avoid collisions on datcom.out / FOR006 / for005
			workDir = tempname;
			mkdir(workDir);

			oldDir = pwd;
			try
				cd(workDir);

				% Write deck into temp dir (so runDatcom copies it to for005 locally)
				deckTmp = fullfile(workDir, 'deck.dcm');
				fid = fopen(deckTmp, 'w');
				fwrite(fid, deckText);
				fclose(fid);

				if ispc
					exePath = fullfile(datcomSourceDir, 'datcom_win.exe');
				elseif ismac && isfile(fullfile(datcomSourceDir, 'datcom_macos'))
					exePath = fullfile(datcomSourceDir, 'datcom_macos');
				else
					exePath = fullfile(datcomSourceDir, 'datcom_unix');
				end

				if ~isfile(exePath)
					error("Syndrome:DATCOM:ExecutableNotFound", ...
						"DATCOM executable not found at '%s'. Expected DATCOM assets under '%s'.", ...
						exePath, datcomSourceDir);
				end

				if ismac
					[statusFile, fileInfo] = system(sprintf('file "%s"', exePath));
					if statusFile == 0 && contains(fileInfo, 'ELF')
						error("Syndrome:DATCOM:UnsupportedBinary", ...
							"DATCOM executable '%s' is reported as '%s', but this machine is macOS (%s). The bundled executable is not expected to run natively on macOS. Build a macOS-compatible DATCOM binary or precompute/cache the aero data elsewhere.", ...
							exePath, strtrim(fileInfo), computer('arch'));
					end
				end

				try
					copyfile(deckTmp, fullfile(pwd, 'for005'));
				catch MEcopy
					error("Syndrome:DATCOM:InputCopyFailed", ...
						"Failed to copy DATCOM deck '%s' into working directory '%s': %s", ...
						deckTmp, pwd, MEcopy.message);
				end

				if ispc
					command = sprintf('echo for005 | "%s"', exePath);
				else
					system(sprintf('chmod +x "%s"', exePath));
					command = sprintf('printf ''for005\n'' | "%s"', exePath);
				end

				[status, cmdout] = system(command);
				if status ~= 0
					if isempty(strtrim(cmdout))
						cmdout = 'No stderr/stdout was returned by the DATCOM process.';
					end
					error("Syndrome:DATCOM:ExecutionFailed", ...
						"DATCOM execution failed using '%s' (status=%d, arch=%s, workdir=%s). %s", ...
						exePath, status, computer('arch'), pwd, strtrim(cmdout));
				end

				outPath = fullfile(workDir, 'datcom.out');
				if ~isfile(outPath)
					error("Syndrome:DATCOM:OutputMissing", ...
						"DATCOM finished but did not create datcom.out in working directory '%s'.", workDir);
				end

				aeroCell = datcomimport(outPath);
				if isempty(aeroCell)
					error("Syndrome:DATCOM:ImportFailed", ...
						"datcomimport returned no aerodynamic cases for '%s'.", outPath);
				end
				aero = aeroCell{1};

				% Save disk cache
				save(cacheFile, 'aero');

			catch ME
				cd(oldDir);
				rocketPkg.DatcomManager.removeLockPath(lockDir);
				rethrow(ME);
			end

			cd(oldDir);
			rocketPkg.DatcomManager.removeLockPath(lockDir);

			% Populate memory cache
			memCache(key) = aero;
		end

		function paths = getDatcomPaths()
			thisFile = mfilename('fullpath');
			rocketPkgDir = fileparts(thisFile);
			srcDir = fileparts(rocketPkgDir);
			projectRoot = fileparts(srcDir);

			paths.projectRoot = projectRoot;
			paths.srcRoot = srcDir;
			paths.datcomSourceDir = fullfile(srcDir, 'DATCOM');
			paths.cacheRoot = fullfile(projectRoot, 'DATCOM');
		end

		function exePath = getDatcomExecutable(paths)
			if ispc
				exePath = fullfile(paths.datcomSourceDir, 'datcom_win.exe');
			elseif ismac && isfile(fullfile(paths.datcomSourceDir, 'datcom_macos'))
				exePath = fullfile(paths.datcomSourceDir, 'datcom_macos');
			else
				exePath = fullfile(paths.datcomSourceDir, 'datcom_unix');
			end

			if ~isfile(exePath)
				error("Syndrome:DATCOM:ExecutableNotFound", ...
					"DATCOM executable not found at '%s'. Expected DATCOM assets under '%s'.", ...
					exePath, paths.datcomSourceDir);
			end

			if ismac
				[status, fileInfo] = system(sprintf('file "%s"', exePath));
				if status == 0 && contains(fileInfo, 'ELF')
					error("Syndrome:DATCOM:UnsupportedBinary", ...
						"DATCOM executable '%s' is reported as '%s', but this machine is macOS (%s). The bundled executable is not expected to run natively on macOS. Build a macOS-compatible DATCOM binary or precompute/cache the aero data elsewhere.", ...
						exePath, strtrim(fileInfo), computer('arch'));
				end
			end
		end

		function tf = isStaleLock(lockDir, staleLockSeconds)
			tf = false;
			if ~exist(lockDir, 'dir')
				return;
			end

			info = dir(lockDir);
			if isempty(info)
				return;
			end

			lockAgeSeconds = (now - info(1).datenum) * 86400;
			tf = lockAgeSeconds > staleLockSeconds;
		end

		function writeLockMetadata(lockMetaFile)
			fid = fopen(lockMetaFile, 'w');
			if fid == -1
				return;
			end

			fprintf(fid, 'pid=%d\n', feature('getpid'));
			hostName = getenv('HOSTNAME');
			if isempty(hostName)
				hostName = 'unknown';
			end
			fprintf(fid, 'host=%s\n', hostName);
			fprintf(fid, 'created=%s\n', datestr(now, 31));
			fclose(fid);
		end

		function msg = formatSystemOutput(cmdout)
			if isempty(strtrim(cmdout))
				msg = 'No stderr/stdout was returned by the DATCOM process.';
			else
				msg = strtrim(cmdout);
			end
		end

		function [deckText, precomputedFile, cacheFile] = getAeroFileTargets(datcomFilename, precomputedAeroDir)
			deckText = fileread(datcomFilename);
			[precomputedFile, cacheFile] = rocketPkg.DatcomManager.getAeroTargetsFromDeckText(deckText, precomputedAeroDir);
		end

		function [precomputedFile, cacheFile] = getAeroTargetsFromDeckText(deckText, precomputedAeroDir)
			[checksum, byteCount] = rocketPkg.DatcomManager.computeDeckSignature(deckText);

			datcomPaths = rocketPkg.DatcomManager.getDatcomPaths();
			cacheDir = fullfile(datcomPaths.cacheRoot, 'cache');
			precomputedDir = char(precomputedAeroDir);
			if isempty(precomputedDir)
				precomputedDir = fullfile(datcomPaths.cacheRoot, 'precomputed');
			end

			cacheFile = fullfile(cacheDir, sprintf('aero_%08X_%d.mat', checksum, byteCount));
			precomputedFile = fullfile(precomputedDir, sprintf('aero_%08X_%d.mat', checksum, byteCount));
		end

		function [checksum, byteCount] = computeDeckSignature(deckText)
            deckText = char(deckText);
			bytes = uint8(deckText);
			checksum = uint32(0);
			for i = 1:numel(bytes)
				checksum = checksum + uint32(bytes(i)) * uint32(mod(i, 65535) + 1);
			end
			byteCount = numel(bytes);
		end

		function savePrecomputedAero(precomputedFile, aero, deckText)
			precomputedDir = fileparts(precomputedFile);
			if ~exist(precomputedDir, 'dir')
				mkdir(precomputedDir);
			end

			save(precomputedFile, 'aero', 'deckText');
		end

		function removeLockPath(lockPath)
			if exist(lockPath, 'dir')
				rmdir(lockPath, 's');
			elseif exist(lockPath, 'file')
				delete(lockPath);
			end
		end

        function deleteIfExists(pathStr)
            if exist(pathStr, 'file')
                delete(pathStr);
            end
        end
    end
end
