classdef (Abstract) Controller < handle&Exportable 
    % For export: This will probably need a wrap once is complete, see 
    % ExportableREADME.txt 2.b.. It is because Controller takes another class. 
    % Example can be found under Example section.
    properties (SetAccess = private)
        actuator
    end

    methods (Abstract)
        % comments
        [F_b, M_b] = get_forces_and_moments(state, disturbances, motor_params, CG, breakPoints, time);
        gain = compute_gain()
    end

    methods 
        function restoreFromStruct(obj, s)
        % Restore all properties from a struct (bypasses SetAccess=private)
            f = fieldnames(s); % Get all field names from the struct 's'
            for i = 1:numel(f) % Loop thru each field
                try
                    obj.(f{i}) = s.(f{i}); % Assign struct to the object property
                catch ME % Warning if fails so it doesn't crash
                    warning('Could not restore %s: %s', f{i}, ME.message);
                end
            end
        end % function added by Zhanna S

    end
end
