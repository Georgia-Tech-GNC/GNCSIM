classdef Exportable < handle
    % Exportable - Base class that provides automatic export functionality
    % 
    % This class allows any inheriting class to automatically export all
    % properties (including private ones) to a timestamped folder as a .mat file.
    %
    % Usage:
    %   - Make your class inherit from Exportable: classdef MyClass < Exportable
    %   - Call obj.exportToMat('MyData') to export to timestamped folder
    %   - Use Exportable.loadFromMat('MyClass', 'path/to/file.mat') to reload
    
    methods
        function exportToMat(obj, baseName, varargin)
            % exportToMat - Export object properties to timestamped .mat file
            %
            % Inputs:
            %   baseName - Base name for the file (string)
            %   basePath - (Optional) Path where timestamped folder should be created
            %
            % Creates:
            %   - Timestamped folder: [basePath/]Export_yyyy-mm-dd_HH/
            %   - .mat file: baseName.mat containing objStruct and metadata
            %
            % Usage:
            %   obj.exportToMat('MyData')                    % Creates in current directory
            %   obj.exportToMat('MyData', 'C:\MyExports')    % Creates in specified path
            
            % Parse optional basePath argument
            if nargin > 2
                basePath = varargin{1};
            else
                basePath = pwd; % Use current directory if no path specified
            end
            
            % Create timestamped folder (day and hour only)
            timestamp = datestr(now, 'yyyy-mm-dd_HH');
            folderName = sprintf('Export_%s', timestamp);
            fullFolderPath = fullfile(basePath, folderName);
            
            % Create folder if it doesn't exist
            if ~exist(fullFolderPath, 'dir')
                mkdir(fullFolderPath);
            end
            
            % Export object to struct
            objStruct = obj.toStruct();
            
            % Create metadata
            metadata.className = class(obj);
            metadata.exportTime = datetime('now');
            metadata.matlabVersion = version;
            metadata.baseName = baseName;
            
            % Save to .mat file
            fileName = fullfile(fullFolderPath, [baseName '.mat']);
            save(fileName, 'objStruct', 'metadata');
            
            fprintf('Exported %s to %s\n', baseName, fileName);
        end
        
        function s = toStruct(obj)
            % toStruct - Convert object properties to struct
            %
            % Returns:
            %   s - Struct containing all accessible object properties
            
            % Get all properties (including private)
            mc = metaclass(obj);
            props = mc.PropertyList;
            
            s = struct();
            for i = 1:length(props)
                propName = props(i).Name;
                try
                    % Access property value (works for private properties too)
                    s.(propName) = obj.(propName);
                catch ME
                    % Skip if property can't be accessed, but warn user
                    warning('Could not access property %s: %s', propName, ME.message);
                end
            end
        end
    end
    
    methods (Static)
        function obj = loadFromMat(className, matFilePath)
            % loadFromMat - Load object from .mat file
            %
            % Inputs:
            %   className - Name of the class to instantiate (string)
            %   matFilePath - Path to the .mat file (string)
            %
            % Returns:
            %   obj - Reconstructed object with restored properties
            %
            % Usage:
            %   rocket = Exportable.loadFromMat('Rocket', 'RocketData_2024-01-15_14-30-25/RocketData.mat');
            
            % Load data from .mat file
            try
                data = load(matFilePath);
            catch ME
                error('Could not load .mat file %s: %s', matFilePath, ME.message);
            end
            
            % Check if required data exists
            if ~isfield(data, 'objStruct')
                error('No objStruct found in .mat file %s', matFilePath);
            end
            
            % Display metadata if available
            if isfield(data, 'metadata')
                fprintf('Loading %s exported on %s\n', ...
                    data.metadata.className, data.metadata.exportTime);
                
                % Verify class name matches
                if ~strcmp(data.metadata.className, className)
                    warning('Class name mismatch: expected %s, found %s', ...
                        className, data.metadata.className);
                end
            end
            
            % Create new instance of the class
            try
                obj = eval([className '()']);
            catch ME
                error('Could not create instance of class %s: %s', className, ME.message);
            end
            
            % Restore properties
            objStruct = data.objStruct;
            fields = fieldnames(objStruct);
            %successCount = 0; commented out because it's irrelevant atm
         
            
            % After constructing obj
            if ismethod(obj, 'restoreFromStruct')
                obj.restoreFromStruct(objStruct);
            else
                % fallback: generic assignment loop
                fields = fieldnames(objStruct);
                for i = 1:length(fields)
                    try
                        obj.(fields{i}) = objStruct.(fields{i});
                    catch ME
                        warning('Could not restore property %s: %s', fields{i}, ME.message);
                    end
                end
            end
            
            %fprintf('Successfully restored %d of %d properties\n',
            %successCount, length(fields)); commented out because it's irrelevant atm
        end
    end
end