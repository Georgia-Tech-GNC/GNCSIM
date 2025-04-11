classdef Gyro
    % Gyro Gyro object. Returns an ideal
    % gyro reading by default. All units are in rad/s.
    
    properties
        bias (1,3) double = [0, 0, 0]; % constant bias (m/s^2)
        range (1,3) double = [Inf, Inf, Inf]; % max measurement range (m/s^2)
        noise_density (1,3) double = [0, 0, 0]; % power spectral density of noise ((m/s^2)/sqrt(Hz))
        sampling_rate (1,1) double = 100; % gyroscope sampling rate (Hz) - Vary this to vary the time-step
        sampling_dt (1,1) double

    end


    methods

        function obj = Gyro(bias, range, noise_density, sampling_rate)
            obj.bias = bias;
            obj.range = range;
            obj.noise_density = noise_density;
            obj.sampling_rate = sampling_rate;

            obj.sampling_dt = 1/obj.sampling_rate;
        end

        function sensed_w_bf = gyro_model(obj, w_bf, t_ref)
        % sensed_w_bf Emulates raw gyroscope readings given a reference 
        % angular rate. Can generate readings for N number of time steps.
        %
        % Inputs:
        %   w_bf (N,3 double): Reference angular rate of body wrt flat earth
        %   gyro_params (struct): Gyroscope parameters
        %
        % Outputs:
        %   sensed_w_bf (N,3 double): Raw gyroscope reading
        %t_
        % Last revised: MM/DD/YYYY
        % Last author: JVR Prasad
        
        % TODO
        % NOTES
        % add non-zero bias and bias rate    

            sensed_w_bf = zeros(length(t_ref),3);
            t_sampling = 0:obj.sampling_dt:t_ref(end);

            w_bf_bias = obj.bias;

            for i = 1:length(t_sampling)
                % Get current sample
                w_bf_true = w_bf(i,:);

                % Generate noise
                w_bf_std = obj.noise_density .* sqrt(obj.sampling_rate);
                w_bf_noise = w_bf_std .* randn([1,3]);

                % Generate bias
                w_bf_bias_new = w_bf_bias + (w_bf_std.*sqrt(obj.sampling_dt)).*w_bf_noise;

                % Compute raw sensor reading
                sensed_w_bf(i,:) = w_bf_true + 0.5.*(w_bf_bias_new + w_bf_bias) + sqrt((w_bf_std.^2)./obj.sampling_dt + (1/12)*(w_bf_std.^2).*obj.sampling_dt).*w_bf_noise;
                w_bf_bias = w_bf_bias_new;

            end
        
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