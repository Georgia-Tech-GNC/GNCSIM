% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

% "Abstract" Controller Class"
% (MATLAB doesn't handle type hinting with real abtract classes)

classdef Controller < handle

    % properties (Abstract)
    %     breakPoints controllerPkg.BreakPoints
    %     referenceAttitudes (:,:) double % ((1+m)xn) time and attitude (m) for every timestep (n).
    %     useActuatorModel (1,1) logical
    % end

    methods
        function u = getControl(obj, state, currentCumVelocity, currentTime) %#ok<STOUT,INUSD>
            error('Controller:getControl', ...
                  'getControl must be implemented in a subclass.');
        end

        function populateBreakPoints(obj) 
            error('Controller:populateBreakPoints', ...
                  'populateBreakPoints must be implemented in a subclass.');
        end
    end

    methods (Static)
        function check_controllability(A, B)
            arguments
                A (:,:) double
                B (:,:) double
            end
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


      
