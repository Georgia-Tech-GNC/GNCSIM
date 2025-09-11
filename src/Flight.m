
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
            
            obj.rocket_dyn.calc_moi()
        end

        function calc_moi()
            
        end

        function run_sim(x0)
            pos = x0(1:3);
            vel = x0(4:6);
            quat = x0(7:10);
            omega = x0(11:13);
        end

        function traj_x = propigate_dynamics(obj, x0)
            % This function propigates the ODE

            traj_x = x0';
            x = x0';
            for ii = 2:length(tspan)
                k1 = rocketModel(x);
                k2 = rocketModel(x + (k1')*obj.dt/2);
                k3 = rocketModel(x + (k2')*obj.dt/2);
                k4 = rocketModel(x + (k3')*obj.dt);

                x_next = x + obj.dt*((1/6)*k1' + (1/3)*k2' + (1/3)*k3' + (1/6)*k4');
                traj_x = [traj_x; x_next];
                x = x_next;
            end
        end

        function x_dot = rocket_model(obj, state)
            h = state(3);
            disturbances.wind = obj.environment.get_wind(h);
            
            
            [F_b, M_b] = obj.rocket_dyn.get_forces_moments(state, disturbances);
            
            x_dot = obj.flat_earth_EOM(state, F_b, M_b);
        end

        function flat_earth_EOM(obj, state, F_b, M_b)
            % TODO: Add flat earth EOM
            
            
        end
    end
end
