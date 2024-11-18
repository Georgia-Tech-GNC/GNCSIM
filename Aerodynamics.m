classdef Aerodynamics
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        FAx_func;
        FAy_func;
        FAz_func;
        n_a_func;
        n_r_func;
        L_p_func;
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

        function obj= set_n_a_func(obj, na_function)
            obj.n_a_func = na_function;
        end

        function obj= set_n_r_func(obj, nr_function)
            obj.n_r_func = nr_function;
        end

        function obj= set_L_p_func(obj, Lp_function)
            obj.L_p_func = Lp_function;
        end

        % Getters
        function FAx_function = get_FAx_func(obj)
            FAx = obj.FAx_func;
        end

        function FAy_function = get_FAy_func(obj)
            FAy = obj.FAy_func;
        end

        function FAz_function = get_FAz_func(obj)
            FAz = obj.FAz_func;
        end

        function na_function = get_n_a_func(obj)
            na_function = obj.n_a_func;
        end

        function nr_function = get_n_r_func(obj)
            nr_function = obj.n_r_func;
        end

        function Lp_function = get_L_p_func(obj)
            Lp_function = obj.L_p_func;
        end

        function [FAb, MAb]= calcAeroBodyFrame_JV(obj, u, v, w, p, r, rho, CG)
            % Inputs [u v w] in (m/s). Inputs [p r] in (rad/s). Input rho
            % in kg/m^3. Input CG in m from tip of nosecone.

            % Aerodynamic forces
            FAx_function = obj.FAx_func;
            FAy_function = obj.FAy_func;
            FAz_function = obj.FAz_func;

            FAx = FAx_function(CG, r, rho, u, v, w);
            FAy = FAy_function(CG, r, rho, u, v, w);
            FAz = FAz_function(u);
            
            FAb = [FAx; FAy; FAz];

            % Aerodynamic moments
            na_function = obj.n_a_func;
            nr_function = obj.n_r_func;
            Lp_function = obj.L_p_func;

            n_a = na_function(rho, u, v, w);
            n_r = nr_function(CG, r, rho, u, v, w);
            L_p = Lp_function(p, u, v, w);

            MAb = [L_p; 0; n_a + n_r];
        end

    end
end



