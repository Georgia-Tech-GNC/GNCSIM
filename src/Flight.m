
classdef Flight
    properties
        rocket_dyn
        controller
        environment
        nav

        flight_params
        sim_length
        dt
    end
    methods
        function obj = Flight(rocket_dyn, controller, environment, nav, flight_params)
            obj.rocket_dyn = rocket_dyn;
            obj.controller = controller;
            obj.environment = environment;
            obj.nav = nav;
            obj.flight_params = flight_params;

            obj.dt = obj.flight_params.dt;
            
            %obj.rocket_dyn.calc_moi()
        end

        %function calc_moi()
            
        %end

        function [t_arr, x_arr] = run_sim(obj, x0)
            mass = x0(1);
            pos = x0(2:4);
            vel = x0(5:7);
            quat = x0(8:11);
            omega = x0(12:14);
            
            N = 100;
            t_arr = zeros(1,N);
            x_arr = zeros(14, N);

            x = x0;
            t = 0;
            for ii=1:N
                x = obj.propagate_dynamics(x);
                t_arr(ii) = t;
                x_arr(:, ii) = x;
                
                t = ii * obj.dt;
            end
        end

        function [t_arr, x_arr] = propagate_dynamics(obj, x0)
            % This function propagates the dynamics using RK4

            x = x0';
            N = 100;

            t_arr = zeros(1,N);
            x_arr = zeros(14, N);

            for ii = 2:N
                k1 = obj.rocket_model(x);
                k2 = obj.rocket_model(x + (k1)*obj.dt/2);
                k3 = obj.rocket_model(x + (k2)*obj.dt/2);
                k4 = obj.rocket_model(x + (k3)*obj.dt);
                
                t = ii * obj.dt;
                x = x + obj.dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

                t_arr(ii) = t;
                x_arr(:, ii) = x;
            end
        end

        function x_dot = rocket_model(obj, state)
            h = state(4);
            %disturbances.wind = obj.environment.get_wind(h);  TODO
            disturbances.wind = 0;
            
            [F_b, M_b] = obj.rocket_dyn.get_forces_and_moments(state, disturbances);
            
            x_dot = obj.flat_earth_EOM(state, F_b, M_b);
        end

        function [state_dot] = flat_earth_EOM(obj, state, F_b, M_b)   
            mass = state(1);
            %pos = state(2:4);  % not needed
            vel = state(5:7).';
            quat = state(8:11).';
            omega = state(12:14).';

            mat_omega = [
                0, -omega(1), -omega(2), -omega(3);
                omega(1), 0, omega(3), -omega(2);
                omega(2), -omega(3), 0, omega(1);
                omega(3), omega(2), -omega(1), 0
            ];

            q = quat;
            C_B_I = [1 - 2 * (q(3)^2 + q(4)^2), 2 * (q(2) * q(3) + q(1) * q(4)), 2 * (q(2) * q(4) - q(1) * q(3));
            2 * (q(2) * q(3) - q(1) * q(4)), 1 - 2 * (q(2)^2 + q(4)^2), 2 * (q(3) * q(4) + q(1) * q(2));
            2 * (q(2) * q(4) + q(1) * q(3)), 2 * (q(3) * q(4) - q(1) * q(2)), 1 - 2 * (q(2)^2 + q(3)^2)];

            C_I_B = C_B_I.';
            g = [0;0;-9.81];

            F_I = (C_I_B * F_b.') + mass * g;
            Ib = obj.rocket_dyn.get_moi(mass);
            
            mass_dot = 0;
            pos_dot = vel;
            vel_dot = F_I / mass;
            quat_dot = 0.5 * mat_omega * quat;
            omega_dot = Ib\(M_b.' - cross(omega, Ib*omega));
            state_dot = [mass_dot; pos_dot; vel_dot; quat_dot; omega_dot].';
        end
    end
end
