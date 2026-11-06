audio_data = read_audio('one_direction_pacer_test');
fs = 47.9e3;
channels = 8;

%% Mix down to baseband and cutoff highest freq
max_freq = 10e3;
mix_tone = exp(1j*2*pi*(-max_freq/2)/fs*(0:length(audio_data)-1)).';
basebandData = audio_data .* mix_tone;
cutoffFrequency = max_freq/2;

Hd = fdesign.lowpass("Fp,Fst,Ap,Ast", cutoffFrequency-200, cutoffFrequency+200, 1, 80, fs);
d = design(Hd, "equiripple");

filtered_data = filter(d, basebandData);

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


%% Compute cross-spectrum
audio_freq = fft(filtered_data);

x_spec = zeros(length(audio_freq), channels, channels);
for channelA = 1:channels
    for channelB = 1:channels
        x_spec(:, channelA, channelB) = audio_freq(:, channelA).*conj(audio_freq(:, channelB));
    end
end

%% Plot cross-spectrum
plot_channels = [1, 7];
figure(5);
ax1 = subplot(2, 1, 1);
plot(10*log10(abs(x_spec(:, plot_channels(1), plot_channels(2)))));
title("Cross-spectrum magnitude")
ylabel("Magnitude (dB)");
ax2 = subplot(2, 1, 2);
plot(angle(x_spec(:, plot_channels(1), plot_channels(2))));
title("Cross-spectrum phase");
xlabel("Frequency bin");
ylabel("Phase (rad)");
linkaxes([ax1, ax2], 'x');

%% Split into smaller sections
fft_size = 256;
if mod(length(filtered_data), fft_size) ~= 0
    filtered_data = [filtered_data; zeros(fft_size - mod(length(filtered_data), fft_size), channels)];
end
data_shaped = reshape(filtered_data, fft_size, [], channels);

%% Compute cross-spectrum for each time domain chunk
data_size = size(data_shaped);
data_spec = zeros(data_size(1), data_size(2), data_size(3), data_size(3));
for channelA = 1:channels
    for channelB = 1:channels
        data_spec(:, :, channelA, channelB) = fft(data_shaped(:, :, channelA)).*conj(fft(data_shaped(:, :, channelB)));
    end
end