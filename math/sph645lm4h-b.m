%
% SPH0645LM4H-B Equalizer filter design
%
% (c)2019 Ivan Kostoski
%

clear;
format long;

% Sampling frequency
Fs = 48000;

% IEC specified frequencies
iec_f = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, ...
         100, 125, 160, 200, 250, 315, 400, 500, 630, 800, ...
         1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
         10000, 12500, 16000, 20000];

% IEC Class 1 tolerances (top/bottom)
iec_c1_t_dB = [+3.5, +3.0, +2.5, +2.5, +2.5, +2.0, +1.5, +1.5, +1.5, +1.5, ...
               +1.5, +1.5, +1.5, +1.5, +1.4, +1.4, +1.4, +1.4, +1.4, +1.4, ...
               +1.1, +1.4, +1.4, +1.6, +1.6, +1.6, +1.6, +2.1, +2.1, +2.1, ...
               +2.6, +3.0, +3.5, +4.0];
iec_c1_b_dB = [-inf, -inf, -4.5, -2.5, -2.5, -2.0, -1.5, -1.5, -1.5, -1.5, ...
               -1.5, -1.5, -1.5, -1.5, -1.4, -1.4, -1.4, -1.4, -1.4, -1.4, ...
               -1.1, -1.4, -1.4, -1.6, -1.6, -1.6, -1.6, -2.1, -2.6, -3.1, ...
               -3.6, -6.0,  -17, -inf];

% IEC Class 2 tolerances (top/bottom)
iec_c2_t_dB = [+5.5, +5.5, +5.5, +3.5, +3.5, +3.5, +2.5, +2.5, +2.5, +2.5, ...
               +2.0, +2.0, +2.0, +2.0, +1.9, +1.9, +1.9, +1.9, +1.9, +1.9, ...
               +1.4, +1.9, +2.6, +2.6, +3.1, +3.1, +3.6, +4.1, +5.1, +5.6, ...
               +6.0, +6.0, +6.0, +6.0];
iec_c2_b_dB = [-inf, -inf, -inf, -3.5, -3.5, -3.5, -2.5, -2.5, -2.5, -2.5, ...
               -2.0, -2.0, -2.0, -2.0, -1.9, -1.9, -1.9, -1.9, -1.9, -1.9, ...
               -1.4, -1.9, -2.6, -2.6, -3.1, -3.1, -3.6, -4.1, -5.1, -5.6, ...
               -inf, -inf, -inf, -inf];

% Values visually estimated from ICS-43434 datasheet 'Typical Frequency Response' plot
        %   10,  12.5,    16,    20,    25,  31.5,    40,    50,   63,    80, ...
ds_dB = [ -inf,  -inf,  -inf,  -7.5,  -6.5,    -5,  -3.6,  -2.8,   -2,  -1.3, ...
            -1,  -0.7,  -0.4,  -0.3,  -0.1,  -0.1,     0,     0,    0,     0, ...
             0,     0,     0,     0,  +0.1,  +0.1,  +0.3,  +0.5, +0.8,  +0.9, ...
            +1,  +0.8,    +0,  -5.5];

% These value are selected and adjusted for better curve fit
ds_l_f  = [  20,   50,  100, 1000];
ds_l_dB = [-7.5, -2.8,   -1,    0];

% Low frequency filter design
% Convert Hz in rad/s and normalize for Fs
ds_l_wn = ds_l_f.*((2*pi)/Fs);
% Convert plot decibels to magnitude
ds_l_mag = arrayfun(@db2mag, ds_l_dB);
% Estimate coefficients
[ds_l_B, ds_l_A] = invfreqz(ds_l_mag, ds_l_wn, 2, 2);
% Stabilize and normalize the filter
ds_l_Bs = polystab(ds_l_B) * norm(ds_l_B) / norm(polystab(ds_l_B));
ds_l_As = polystab(ds_l_A) * norm(ds_l_A) / norm(polystab(ds_l_A));

% High frequency filter design
%ds_h_f  = [ 1000, 2000,  4000, 8000, 10000, 16000, 20000];
%ds_h_dB = [    0,    0,  +0.3, +0.9,    +1,     0,  -5.5];
%[ds_h_B, ds_h_A] = invfreqz(arrayfun(@db2mag, ds_h_dB), (ds_h_f.*((2*pi)/Fs)), 4, 3);
%ds_h_Bs = polystab(ds_h_B) * norm(ds_h_B) / norm(polystab(ds_h_B));
%ds_h_As = polystab(ds_h_A) * norm(ds_h_A) / norm(polystab(ds_h_A));

% Convolve into single 4th order filter
ds_B = ds_l_Bs % conv(ds_l_Bs, ds_h_Bs);
ds_A = ds_l_As % conv(ds_l_As, ds_h_As);
ds_H = freqz(ds_B, ds_A, iec_f, Fs);

% Equalizer filter, i.e. inverse from estimated transfer filter
% Swap A and B coefficients, and normalize to ds_B(1)
eq_B = ds_A./ds_B(1)
eq_A = ds_B./ds_B(1)
% Check for poles ouside unit circle
roots(eq_B)
roots(eq_A)

% Add DC blocking filter
[sos, gain] = tf2sos(eq_B, eq_A)
sos = [sos; [1.0, -1.0, 0.0, 1.0, -0.9992, 0]];
[eq_B, eq_A] = sos2tf(sos, gain)
eq_H = freqz(eq_B, eq_A, iec_f, Fs);

clf;
semilogx(iec_f, ds_dB, 'g;SPH0645LM4H-B Datasheet plot (approx.);');
hold on;
title("SPH0645LM4H-B Frequency response");
grid minor;
xlabel('Frequency (Hz)');
xlim([10, 24000]);
ylabel('Amplitude (dB)');
ylim([-30, 20]);
legend ('boxoff');
legend ('location', 'northwest');
semilogx(iec_f, iec_c1_t_dB, '--r;IEC 61672-1:2013 Class 1 tolerance;');
semilogx(iec_f, iec_c1_b_dB, '--r');
semilogx(iec_f, iec_c2_t_dB, 'r;IEC 61672-1:2013 Class 2 tolerance;');
semilogx(iec_f, iec_c2_b_dB, 'r');
semilogx(iec_f, 20*log10(abs(ds_H)), '--c;IIR filter frequency response;');
semilogx(iec_f, (ds_dB + 20*log10(abs(eq_H))), 'b;Adjusted frequency response;', 'linewidth', 3);
hold off;

% Convert to Second-Order Sections
[sos, gain] = tf2sos(eq_B, eq_A)