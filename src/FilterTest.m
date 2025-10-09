amplitude = 5;          % Amplitude of the sine wave
frequency = 0.5;          % Frequency of the sine wave in Hz
sampling_rate = 200;    % Samples per second
duration = 10;           % Duration of the signal in seconds

% 2. Generate the time vector
t = linspace(0, duration, sampling_rate * duration); % Time vector from 0 to duration

% 3. Create the clean sine wave
signal = amplitude * sin(2 * pi * frequency * t);

% 4. Generate random noise
noise_amplitude = 0.2; % Adjust this value to control the noise level
noise = noise_amplitude * randn(size(t)); % Generate Gaussian white noise

% 5. Add noise to the signal
noisy_signal = signal + noise;

filtered_signals = [];
lpf = LowPassFilter(20);
for i = 1:length(noisy_signal)
    filtered_signals(i) = lpf.filter(noisy_signal(i));
end

plot(t,noisy_signal,t,filtered_signals)


