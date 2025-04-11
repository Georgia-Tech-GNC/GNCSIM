classdef Accelerometer
    % Accelerometer Accelerometer object. Returns an ideal
    % accelerometer reading by default. All units are in m/s^2.
    
    properties

        bias (1,3) double = [0, 0, 0]; % constant bias (m/s^2)
        range (1,3) double = [Inf, Inf, Inf]; % max measurement range (m/s^2)
        noise_density (1,3) double = [0, 0, 0]; % power spectral density of noise ((m/s^2)/sqrt(Hz))
        sampling_rate (1,1) double = 100; % accelerometer sampling rate (Hz)
        sampling_dt (1,1) double
        refTraj;

    end


    methods

        function log = run_ekf(inputs)
            
            


        end

        function obj = Sensor()
            % Constructor
            obj.refTraj = open('data/JV1.mat'); % Need to bring stuff from jet vanes simulations
            
        end


        function sf = accelerometer_reading(obj, a_f, w_bf, q_bf)
        % accelerometer_model Emulates raw accelerometer readings given a reference 
        % state. Current fidelity accounts for lever arm offset from CG. Can 
        % generate readings for N number of time steps.
        %
        % Inputs:
        %   a_f (N,3 double): Inertial acceleration of CG in the flat earth frame
        %   w_bf (N,3 double): Angular rate of body wrt flat earth
        %   q_bf (N,4 double): Quaternion rotation from flat earth to body
        %
        % Outputs:
        %   sf (N,3 double): Specific force in IMU frame
        %
        % Last revised: MM/DD/YYYY
        % Last author: JVR Prasad
        
        % TODO
        % NOTES
        % use state2imu for the IMU lever arm conversion
        % add bias and bias rate
        % variable gravity would be nice
        % include lever arm offset and cg rate in accel_params

        
        [x2f, v2f, a2f] = state2imu(w_bf, x_f, v_f, a_f, q_bf, r21b, v21b);
        

        
        end

        function [x2f, v2f, a2f] = state2imu(wbfb, x1f, v1f, a1f, qbf, r21b, v21b)
   
           qfb = quatconj(qbf'); % outputs 1x4
           r12b = -r21b;
           v12b = -v21b;
        
           % Assumptions
           a21b = [0;0;0];
           wfib = [0;0;0];
           alphabfb = [0;0;0];
           
           x2f = x1f - quat_rotate(qfb, r21b')';
           v2f = v1f - quat_rotate(qfb, (v12b + cross(wbfb,r12b))')';
           a2f = a1f - quat_rotate(qfb, (cross(wbfb,v21b) + cross(wbfb,cross(wbfb,r12b)) )')';
        
        end

        % Setters
        function obj = set_bias(obj,new_bias)
            obj.bias = new_bias;
        end

        function obj = set_noise_density(new_noise_density)
            obj.noise_density = new_noise_density;
        end

        function obj = set_sampling_rate(new_sampling_rate)
            obj.sampling_rate = new_sampling_rate;
        end

        % Getters
        function [bias] = get_bias(obj)
            bias = obj.bias;
        end

        function [noise_density] = get_noise_density(obj)
            noise_density = obj.noise_density;
        end

        function [sampling_rate] = get_sampling_rate(obj)
            sampling_rate = obj.sampling_rate;
        end


    end

end
