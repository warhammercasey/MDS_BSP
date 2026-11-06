audio_data = read_audio('one_direction_tone');
fs = 47.9e3;
channels = 8;

%% Mix down to baseband and cutoff highest freq
max_freq = 10e3;
mix_tone = exp(1j*2*pi*(-max_freq/2)/fs*(0:length(audio_data)-1)).';
basebandData = audio_data .* mix_tone;
cutoffFrequency = max_freq/2;

Hd = fdesign.lowpass("Fp,Fst,Ap,Ast", cutoffFrequency-500, cutoffFrequency+500, 1, 60, fs);
d = design(Hd, "equiripple");

filtered_data = filter(d, basebandData);
filtered_data = filtered_data(1:4:end, :);
fs = fs/4;

%% Plot
figure(1);
plot(0:1/fs:(1/fs*(length(audio_data) - 1)), audio_data);
title("Raw audio");
xlabel("Time (S)");
ylabel("Amplitude");

figure(2);
plot_spectrum(audio_data, fs);
title("Raw audio spectrum");

figure(3);
plot_spectrum(basebandData, fs);
title("Basebanded");

figure(4);
plot_spectrum(filtered_data, fs);
title("Filtered");

%% Correlate each channel
% Max time delay is max_dist/343m/s
% Max distance is distance between the two farthest corners of the 16x16
% cube
sp = 160e-3; % Spacing (m)
soundSpeed = 343; % Speed of sound (m/s)
max_time = sqrt(3)*sp/soundSpeed;
correlation_size = ceil(max_time*fs)*256;
max_diff = ceil(max_time*fs);

% Zero pad to be multiple of correlation size
if mod(length(filtered_data), correlation_size) ~= 0
    filtered_data = [filtered_data; zeros(correlation_size - mod(length(filtered_data), correlation_size), channels)];
end
filtered_data_reshape = reshape(filtered_data, correlation_size, [], channels);

correlated = zeros(correlation_size*2 - 1, size(filtered_data_reshape, 2), channels, channels);
lags = zeros(correlation_size*2 - 1, size(filtered_data_reshape, 2), channels, channels);
for i = 1:size(filtered_data_reshape, 2)
    for channelA = 1:channels
        for channelB = 1:channels
            [correlated(:, i, channelA, channelB), lags(:, i, channelA, channelB)] = xcorr(filtered_data_reshape(:, i, channelA), filtered_data_reshape(:, i, channelB));
            correlated(abs(lags(:, i, channelA, channelB)) > max_diff, i, channelA, channelB) = 0;
        end
    end
end


%% Compute delay from max correlation index
[~, peakIndices] = max(abs(correlated), [], 1);
delaySamples = squeeze(peakIndices - correlation_size);
delaySeconds = delaySamples / fs;

%% Microphone locations
sp = 160e-3/2; % Spacing (m)
mic_locations = sp*[-1, -1, -1, -1,  1,  1,  1,  1;
                    -1,  1, -1,  1, -1,  1, -1,  1;
                    -1, -1,  1,  1,  1,  1, -1, -1];

%% Estimate direction with least-squares
soundSpeed = 343; % Speed of sound (m/s)
num_compares = channels*(channels-1)/2;
delays_1d = zeros(num_compares, length(delaySeconds));
A = zeros(num_compares, 3);

idx = 1;
for channelA = 1:channels-1
    for channelB = (channelA+1):channels
        delays_1d(idx, :) = delaySeconds(:, channelB, channelA).';
        A(idx, :) = (mic_locations(:, channelA) - mic_locations(:, channelB)).';
        idx = idx + 1;
    end
end

directionVector = (A \ (soundSpeed*delays_1d));


%% Plot direction arrow
i = 1500:2500;
figure(5);
quiver3(zeros(1, length(directionVector)), zeros(1, length(directionVector)), zeros(1, length(directionVector)), directionVector(1, :), directionVector(2, :), directionVector(3, :));
%quiver3(0, 0, 0, mean(directionVector(1, :)), mean(directionVector(2, :)), mean(directionVector(3, :)));

%% Time domain moving average
movavg = movmean(directionVector.', 20);

figure(6);
quiver3(zeros(length(movavg), 1), zeros(length(movavg), 1), zeros(length(movavg), 1), movavg(:, 1), movavg(:, 2), movavg(:, 3));