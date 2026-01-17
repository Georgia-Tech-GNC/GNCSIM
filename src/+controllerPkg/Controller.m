% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef (Abstract) Controller < handle
    properties (Constant)
        idx = StateIndices
    end

    properties (Abstract)
        breakPoints
        referenceAttitudes % (5xn) time and quaternion for every timestep.
        useActuatorModel
    end

    methods (Abstract)
        u = getControl(obj, state);
        % Get control input based on state and breakpoints

        populateBreakPoints(obj);
        % Populate breakPoints with A, B, and gain(s)
    end

    methods (Static)
        function checkControllability(A, B)
            C = ctrb(A, B);
            r = rank(C);
            n = size(A, 1);
    
            fprintf('rank(C) = %d\n', r);
            fprintf('n       = %d\n', n);
    
            if r == n
                fprintf('System is CONTROLLABLE.\n');
            else
                fprintf('System is NOT controllable.\n');
    
                k = n - r;  % number of uncontrollable modes
    
                % Uncontrollable subspace basis
                N = null(C');   % n x k
    
                for i = 1:k
                    fprintf('\nUncontrollable mode z%d = v%d^T x\n', i, i);
                    fprintf('v%d = [\n', i);
                    disp(N(:, i))
                end
    
                % Optional: show dominant states
                fprintf('Absolute contribution of each state:\n');
                disp(abs(N))
            end
        end
    end
end




      
