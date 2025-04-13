classdef Magnetometer
    % Magnetometer Magnetometer object. Returns an ideal
    % magnetometer reading by default. All units are in microTesla.
    
    properties
        bias (1,3) double = [0, 0, 0]; % constant bias (microT)
        range (1,3) double = [Inf, Inf, Inf]; % max measurement range (microT)
        noise_density (1,3) double = [0, 0, 0]; % power spectral density of noise (microT/sqrt(Hz))
        sampling_rate (1,1) double = 100; % magnetometer sampling rate (Hz)
    
        coarse_lla (1,3) double =  [33.776, -84.399, 306]; % LLA of Georgia Tech
        
        % Reference: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        coarse_declination (1,1) double = 5.52; % Magnetic declination at Georgia Tech (deg)

    end

    properties (Dependent)
        sampling_dt (1,1) double
    end

    methods

        function obj = Magnetometer(bias, range, noise_density, sampling_rate)
            % TODO use varargin and inputParser

            obj.bias = bias;
            obj.range = range;
            obj.noise_density = noise_density;
            obj.sampling_rate = sampling_rate;
            obj.coarse_lla = coarse_lla;

            obj.sampling_dt = 1/obj.sampling_rate; % magnetometer sampling time step (sec)

        end

        function m_b = mag_reading(obj, p_ecef, q_bf, t_ref)
        % mag_reading Emulates raw magnetometer readings given a reference
        % position and attitude. Can generate readings for N number of time steps.
        %
        % Inputs:
        %   p_ecef (N,3 double): Position of sensor in ECEF frame
        %   q_bf (N,4 double): Quaternion that rotates from flat earth to body
        %   frame
        %   t_ref (N,1 double): Associated reference time profile
        %   mag_params (MagParams): Struct of magnetometer parameters
        %
        % Outputs:
        %   m_b (N,3 double): Raw magnetometer reading in sensor frame
        %   (microT)
            
            m_b = zeros(length(t_ref),3);
            t_sampling = 0:obj.sampling_dt:t_ref(end);

            % Sample reference data
            for i = 1:length(t_sampling)

                % Get current sample
                curr_p_ecef = interp1(t_ref,p_ecef,t_sampling(i));
                curr_q_bf = interp1(t_ref,q_bf,t_sampling(i)); % TODO use slerp instead, more accurate

                % Convert ECEF to LLA
                lla = ecef2lla(curr_p_ecef); % NOTE: this function defeines altitude as above the planetary ellipsoid
                lat = lla(:,1);
                lon = lla(:,2);
                alt = lla(:,3);
                
                % Get magnetic field in NED frame
                [m_v,~,~,~,~] = wrldmagm(alt, lat, lon, 2025); % nT
                m_v = m_v'; % 1x3
                m_v = m_v ./ 1e3; % nT to microT

                % Rotate NED to body frame
                m_b_true = quat_rotate(curr_q_bf,m_v);

                % Generate noise
                m_b_std = obj.noise_density .* sqrt(obj.sampling_rate);
                m_b_noise = m_b_std .* randn([1,3]);

                % Compute raw sensor reading
                m_b(i,:) = m_b_true + obj.bias + m_b_noise;

               
            end
            
        end

%         function N_b = magnetic_declination(m_b,p_ecef)
% 
%             arguments
%                 N_b (N,3 double): True north in sensor frame
%             end
% 
%         end

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

        function obj = set_coarse_lla(new_coarse_lla)
            obj.coarse_lla = new_coarse_lla;
        end

        function obj = set_coarse_declination(new_coarse_declination)
            obj.coarse_declination = new_coarse_declination;
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
           
        function [coarse_lla] = get_coarse_lla(obj)
            coarse_lla = obj.coarse_lla;
        end

        function [coarse_declination] = get_coarse_declination(obj)
            coarse_declination = obj.coarse_declination;
        end

    end
    
end