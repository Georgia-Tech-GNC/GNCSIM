% Sine Wave Parameters
amplitude = 2;          % Amplitude of the sine wave
frequency = 0.5;        % Frequency of the sine wave in Hz
sampling_rate = 200;    % Samples per second
duration = 10;          % Duration of the signal in seconds

% Generate the time vector
t = linspace(0, duration, sampling_rate * duration); 

% Create the clean sine wave
signal = amplitude * sin(2 * pi * frequency * t);

% Noise Generation
noise_amplitude = 0.2; % Adjust this value to control the noise level
noise = noise_amplitude * randn(size(t)); % Generate Gaussian white noise

% Add Noise to Signal
noisy_signal = signal + noise;

% Iterative filtering, meant to simulate live data
filtered_signals = [];
lpf = LowPassFilter(10);
for i = 1:length(noisy_signal)
    filtered_signals(i) = lpf.filter(noisy_signal(i));
end

% Plot unfiltered and filtered data
plot(t,noisy_signal,t,filtered_signals)


