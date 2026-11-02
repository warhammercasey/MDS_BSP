function plot_spectrum(data,fs)
freq = linspace(-fs/2, fs/2, length(data));
fft_data = 20*log10(abs(fftshift(fft(data)))/length(data));
plot(freq, fft_data);
xlabel("Frequency (Hz)");
ylabel("Magnitude (dBFS)");
end