Exportable.m is meant to make custom classes... exportable (duh). If your custom class
has 'properties (SetAccess = private)', you will not be able to export without the correct
implementation of the class.

To use Exportable.m properties on a custom class set with private access, the following 
must be true:

1. Your class must inherit Exportable

2. a. Your class must have the following:
	
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
        end

   b. Your class, if it DOES contain other classes or inputs, must have the same as all above.
      And in addition, you must wrap the obj. SEE EXAMPLE below...
		
________________________________________________________________________________________

EXAMPLE:

1. Attach '< Exportable' on the line where the class is defined. If it already inherits from another class,
just use '&Exportable'. It's pretty straight forward...

   classdef ClassName                      becomes            classdef ClassName < Exportable
   classdef Environment < handle           becomes           classdef Environment < handle&Exportable

2. a. Under methods, insert the provided function. Make sure your obj is defined. If not, create an empty 
instance. Remember that there could only be one instance.

classdef Aerodynamics < handle&Exportable

    properties (SetAccess = private)
    blahblahblah
    end

    methods
        function obj = Aerodynamics()
            % Construct an instance of this class
        end

 +      function restoreFromStruct(obj, s)
 +      % Restore all properties from a struct (bypasses SetAccess=private)
 +          f = fieldnames(s); % Get all field names from the struct 's'
 +          for i = 1:numel(f) % Loop thru each field
 +              try
 +                  obj.(f{i}) = s.(f{i}); % Assign struct to the object property
 +              catch ME % Warning if fails so it doesn't crash
 +                  warning('Could not restore %s: %s', f{i}, ME.message);
 +              end
 +          end
 +      end
        
        setters&getters functions will follow

    end
end

2. b. Follow all the steps above first! Below, is an example (all under methods). Make sure you assign varargin{};
to every value you see in the original input:

        function obj = TrimPoints(file_path) % here, we will replace file_path with varargin
            blahblahblah
        end



becomes



*       function obj = TrimPoints(varargin)
+           if nargin == 0
+               % Empty constructor for loading from MAT
+               return
+
+           elseif nargin == 1 % because we only had 1 input
+               file_path = varargin{1}; % make sure you "count" every input that you had originally by assigning it to varargin{};
                blahblahblah
+           else
+               error('TrimPoints constructor expects either 0 or 1 input argument.');
+           end
        end 