% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef FullStateLQR < controllerPkg.Controller
    properties
        breakPoints 
        referenceAttitudes 
        Q (:,:) double
        R (:,:) double
        useActuatorModel = false
        % reduceLTI controllerPkg.ReduceLTI
    end

    methods
        function obj = FullStateLQR(breakPoints, referenceAttitudes, Q, R)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.Q = Q;
            obj.R = R;
            % obj.reduceLTI = reduceLTI;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            arguments
                obj
                state (:,1) double
                currentCumVelocity (1,1) double {mustBeNonnegative}
                currentTime (1,1) double {mustBeNonnegative}
            end
            [interpolatedGains, interpolatedControlReference] = obj.breakPoints.interpolateGain(currentCumVelocity);

        end
           
        function [rollErr, pitchErr, yawErr] = getError(obj, rocketState, currentTime)
            arguments
                obj
                rocketState (:,1) double
                currentTime (1,1) double {mustBeNonnegative}
            end

        end
        
        function populateBreakPoints(obj)
            % Only call this function before running flight
            for i = 1:obj.breakPoints.numBreakPoints
                time = obj.breakPoints.time(i);
                state = obj.breakPoints.state{i};
                controlInput = obj.breakPoints.controlInput{i};

                [A3, B3] = obj.reduceLTI.calculateROMpitch(state, controlInput(2), time);
                [A2, B2] = obj.reduceLTI.calculateROMroll(state, controlInput(1), time);

                K3 = lqr(A3, B3, obj.Q3, obj.R3);
                K2 = lqr(A2, B2, obj.Q2, obj.R2);

                obj.breakPoints.stateMatrix{i} = {A2, A3};
                obj.breakPoints.controlMatrix{i} = {B2, B3};
                obj.breakPoints.gain{i} = {K2, K3};
            end
        end
    end

end