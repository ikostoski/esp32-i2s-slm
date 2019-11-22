%
% INMP441 Equalizer filter design
%
% (c)2019 Ivan Kostoski
%

clear;
% Sampling frequency
Fs = 48000;

% IEC specified frequencies
iec_f = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, ...
         100, 125, 160, 200, 250, 315, 400, 500, 630, 800, ...
         1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
         10000, 12500, 16000, 20000];

% Values visually estimated from INMP441 datasheet 'Typical Frequency Response' plot
        %  10, 12.5,   16,   20,   25, 31.5,   40,   50,   63,   80, ...
ds_dB = [ -inf, -inf, -inf, -12,  -10,   -8,   -6, -4.4, -3.4, -2.5, ...
         -1.9, -1.4, -0.9, -0.5, -0.2,    0,    0,    0,    0,    0, ...
            0,   0,     0,    0,    0,    0,  -0.4, -0.7, -1.1, -1.8, ...
         -2.4, -3.0, -5.0, -inf];

% These value are selected and adjusted for better curve fit
ds_l_f  = [  20,   50,  100, 1000];
ds_l_dB = [ -13, -4.1, -1.8,    0];

% Low frequency filter design
% Convert Hz in rad/s and normalize for Fs
ds_l_w = ds_l_f.*((2*pi)/Fs);
% Convert plot decibels to magnitude
ds_l_mag = arrayfun(@db2mag, ds_l_dB);
% Estimate coefficients
[ds_l_B, ds_l_A] = invfreqz(ds_l_mag, ds_l_w, 2, 2);
% Stabilize and normalize the filter
ds_A = polystab(ds_l_A) * norm(ds_l_A) / norm(polystab(ds_l_A));
ds_B = polystab(ds_l_B) * norm(ds_l_B) / norm(polystab(ds_l_B));
ds_H = freqz(ds_B, ds_A, iec_f, Fs);

% Equalizer filter, i.e. inverse from estimated transfer filter
% Swap A and B coefficients, and normalize to ds_B(1)
eq_B = ds_A./ds_B(1)                                         
eq_A = ds_B./ds_B(1)
eq_H = freqz(eq_B, eq_A, iec_f, Fs);

clf;
figure(1, 'position', [0,0,800,500]);
title("INMP441 Frequency response");
grid minor;
xlabel('Frequency (Hz)');
xlim([10, 20000]);
ylabel('Magnitude (dB)');
ylim([-20, 20]);
hold on;
semilogx(iec_f, ds_dB, 'g;INMP441 Datasheet plot (approx.);');
semilogx(iec_f, 20*log10(abs(ds_H)), '--c;IIR filter frequency response;');
semilogx(iec_f, (ds_dB + 20*log10(abs(eq_H))), 'b;Adjusted frequency response;', 'linewidth', 3);
legend ('boxoff');
legend ('location', 'northwest');
hold off;

[sos, gain] = tf2sos(eq_B, eq_A)