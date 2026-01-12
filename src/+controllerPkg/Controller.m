% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef (Abstract) Controller < handle

    properties (Abstract)
        breakPoints
        referenceAttitudes % (5xn) time and quaternion for every timestep.
    end

    methods (Abstract)
        u = getControl(obj, state);
        % Get control input based on state and breakpoints

        populateBreakPoints(obj);
        % Populate breakPoints with A, B, and gain(s)
    end

    methods (Static)
        function check_controllability(A, B)
            C = ctrb(A, B);
            r = rank(C);
            n = size(A, 1);
        
            fprintf('rank(C) = %d\n', r);
            fprintf('n       = %d\n', n);
        
            if r == n
                fprintf('System is CONTROLLABLE.\n');
            else
                fprintf('System is NOT controllable.\n');
            end
        end

    end

end


      
