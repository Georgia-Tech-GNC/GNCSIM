% Main Authors: Patrick Barry

% Meant for implementing CFD models of rockets
% Use DATCOM if you don't have one

classdef AerodynamicsCFD < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        idx = StateIndices
    end

    properties (Access = private)
        fAxFunc;
        fAyFunc;
        fAzFunc;
        mAxFunc;
        mAyFunc;
        mAzFunc;
    end

    methods
        function obj = AerodynamicsCFD()
            % Construct an instance of this class

            % TODO Future project: Define a default basic aerodynamic model for
            % concept vehicle development
        end

        % Setters
        function obj = setFAxFunc(obj, fAxFunction)
            obj.fAxFunc = fAxFunction;
        end

        function obj = setFAyFunc(obj, fAyFunction)
            obj.fAyFunc = fAyFunction;
        end

        function obj = setFAzFunc(obj, fAzFunction)
            obj.fAzFunc = fAzFunction;
        end

        function obj = setMAxFunc(obj, mAxFunction)
            obj.mAxFunc = mAxFunction;
        end

        function obj = setMAyFunc(obj, mAyFunction)
            obj.mAyFunc = mAyFunction;
        end

        function obj = setMAzFunc(obj, mAzFunction)
            obj.mAzFunc = mAzFunction;
        end

        % Getters
        function fAxFunction = getFAxFunc(obj)
            fAxFunction = obj.fAxFunc;
        end

        function fAyFunction = getFAyFunc(obj)
            fAyFunction = obj.fAyFunc;
        end

        function fAzFunction = getFAzFunc(obj)
            fAzFunction = obj.fAzFunc;
        end

        function mAxFunction = getMAxFunc(obj)
            mAxFunction = obj.mAxFunc;
        end

        function mAyFunction = getMAyFunc(obj)
            mAyFunction = obj.mAyFunc;
        end

        function mAzFunction = getMAzFunc(obj)
            mAzFunction = obj.mAzFunc;
        end
        
        function [fAb, mAb] = getForcesMoments(obj, state, envConditions, CG)
            % Inputs [u v w] in (m/s). Inputs [p r] in (rad/s). Input rho
            % in kg/m^3. Input CG in m from tip of nosecone.

            rho = envConditions.Density; %kg/m^3

            cBI = Utils.dirCosine(state(obj.idx.quat));
            wind = cBI * envConditions.WindVector;

            u = state(obj.idx.vx) - wind(1);
            v = state(obj.idx.vy) - wind(2);
            w = state(obj.idx.vz) - wind(3);
            p = state(obj.idx.omega_x);
            r = state(obj.idx.omega_z);


            % Aerodynamic forces
            fAxFunction = obj.fAxFunc;
            fAyFunction = obj.fAyFunc;
            fAzFunction = obj.fAzFunc;

            fAx = fAxFunction(u, v, w, rho, CG, p, r);
            fAy = fAyFunction(u, v, w, rho, CG, p, r);
            fAz = fAzFunction(u, v, w, rho, CG, p, r);

            fAb = [fAx; fAy; fAz];

            % Aerodynamic moments
            mAxFunction = obj.mAxFunc;
            mAyFunction = obj.mAyFunc;
            mAzFunction = obj.mAzFunc;

            mAxB = mAxFunction(u, v, w, rho, CG, p, r);
            mAyB = mAyFunction(u, v, w, rho, CG, p, r);
            mAzB = mAzFunction(u, v, w, rho, CG, p, r);

            mAb = [mAxB; mAyB; mAzB];
        end

    end
end



