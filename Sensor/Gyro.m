classdef Gyro
    % Gyro Gyro object. Returns an ideal
    % gyro reading by default. All units are in rad/s.
    
    properties
        bias (1,3) double = [0, 0, 0]; % constant bias (m/s^2)
        range (1,3) double = [Inf, Inf, Inf]; % max measurement range (m/s^2)
        noise_density (1,3) double = [0, 0, 0]; % power spectral density of noise ((m/s^2)/sqrt(Hz))
        sampling_rate (1,1) double = 100; % accelerometer sampling rate (Hz)
        
        sampling_dt (1,1) double

    end


    methods

        function sensed_w_bf = gyro_model(w_bf, gyro_params)
        % accelerometer_model Emulates raw gyroscope readings given a reference 
        % angular rate. Can generate readings for N number of time steps.
        %
        % Inputs:
        %   w_bf (N,3 double): Reference angular rate of body wrt flat earth
        %   gyro_params (struct): Gyroscope parameters
        %
        % Outputs:
        %   sensed_w_bf (N,3 double): Raw gyroscope reading
        %
        % Last revised: MM/DD/YYYY
        % Last author: JVR Prasad
        
        % TODO
        % NOTES
        % add bias and bias rate
        
        end




    end



end