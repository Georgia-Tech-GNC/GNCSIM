classdef Accelerometer
    % Accelerometer Accelerometer object. Returns an ideal
    % accelerometer reading by default. All units are in m/s^2.
    
    properties
        bias (1,3) double = [0, 0, 0]; % constant bias (m/s^2)
        range (1,3) double = [Inf, Inf, Inf]; % max measurement range (m/s^2)
        noise_density (1,3) double = [0, 0, 0]; % power spectral density of noise ((m/s^2)/sqrt(Hz))
        sampling_rate (1,1) double = 100; % accelerometer sampling rate (Hz)
        
        sampling_dt (1,1) double

    end


    methods

        function sf = accelerometer_reading(obj, a_f, w_bf, q_bf)
        % accelerometer_model Emulates raw accelerometer readings given a reference 
        % state. Current fidelity accounts for lever arm offset from CG. Can 
        % generate readings for N number of time steps.
        %
        % Inputs:
        %   a_f (N,3 double): Inertial acceleration of CG in the flat earth frame
        %   w_bf (N,3 double): Angular rate of body wrt flat earth
        %   q_bf (N,4 double): Quaternion rotation from flat earth to body
        %   accel_params (struct): Accelerometer parameters
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
        
        end




    end



end
