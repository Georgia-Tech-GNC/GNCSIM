classdef Aerodynamics < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        FAx_func;
        FAy_func;
        FAz_func;
        MAx_func;
        MAy_func;
        MAz_func;
    end

    methods
        function obj = Aerodynamics()
            % Construct an instance of this class

            % TODO Future project: Define a default basic aerodynamic model for
            % concept vehicle development
        end

        % Setters
        function obj= set_FAx_func(obj, FAx_function)
            obj.FAx_func = FAx_function;
        end

        function obj= set_FAy_func(obj, FAy_function)
            obj.FAy_func = FAy_function;
        end

        function obj= set_FAz_func(obj, FAz_function)
            obj.FAz_func = FAz_function;
        end

        function obj= set_MAx_func(obj, MAx_function)
            obj.MAx_func = MAx_function;
        end

        function obj= set_MAy_func(obj, MAy_function)
            obj.MAy_func = MAy_function;
        end

        function obj= set_MAz_func(obj, MAz_function)
            obj.MAz_func = MAz_function;
        end

        % Getters
        function FAx_function = get_FAx_func(obj)
            FAx_function = obj.FAx_func;
        end

        function FAy_function = get_FAy_func(obj)
            FAy_function = obj.FAy_func;
        end

        function FAz_function = get_FAz_func(obj)
            FAz_function = obj.FAz_func;
        end

        function MAx_function = get_MAx_func(obj)
            MAx_function = obj.MAx_func;
        end

        function MAy_function = get_MAy_func(obj)
            MAy_function = obj.MAy_func;
        end

        function MAz_function = get_MAz_func(obj)
            MAz_function = obj.MAz_func;
        end

        function [FAb, MAb]= calcAeroBodyFrame_JV(obj, u, v, w, p, r, rho, CG)
            % Inputs [u v w] in (m/s). Inputs [p r] in (rad/s). Input rho
            % in kg/m^3. Input CG in m from tip of nosecone.

            % Aerodynamic forces
            FAx_function = obj.FAx_func;
            FAy_function = obj.FAy_func;
            FAz_function = obj.FAz_func;

            FAx = FAx_function(u, v, w, rho, CG, p, r);
            FAy = FAy_function(u, v, w, rho, CG, p, r);
            FAz = FAz_function(u, v, w, rho, CG, p, r);
            
            FAb = [FAx; FAy; FAz];

            % Aerodynamic moments
            MAx_function = obj.MAx_func;
            MAy_function = obj.MAy_func;
            MAz_function = obj.MAz_func;

            MAx_b = MAx_function(u, v, w, rho, CG, p, r);
            MAy_b = MAy_function(u, v, w, rho, CG, p, r);
            MAz_b = MAz_function(u, v, w, rho, CG, p, r);

            MAb = [MAx_b; MAy_b; MAz_b];
        end

    end
end



