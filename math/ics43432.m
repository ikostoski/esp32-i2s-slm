%
% ICS-43432 Equalizer filter design
%
% (c)2019 Ivan Kostoski
%

clear;
Fs = 48000;
format long G;

% IEC specified frequencies
iec_f = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, ...
         100, 125, 160, 200, 250, 315, 400, 500, 630, 800, ...
         1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
         10000, 12500, 16000, 20000];

% Values visually estimated from ICS-43432 datasheet for IEC frequencies
% https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
% Figure 5 on page 9
%            10, 12.5,   16,   20,   25,  31.5,   40,   50,   63,   80,
ds_e_dB = [ -22,  -19,  -15,  -12,   -8,    -6, -4.5, -3.5, -2.1, -1.1, ...
           -0.4, +0.1, +0.6, +0.9, +1.0,   0.9, +0.8, +0.6, +0.4, +0.3, ...
              0, +0.3,  +0.4, +0.5, +0.7, +1.0, +1.2, +1.4, +1.8, +2.3, ...
           +3.5, +6.0,  +9.0, +14.0];

% Manually chosen and adjusted values for fitting the transfer function
ds_f  = [ 20, 100,   630, 1150, 2000, 10000, 20000];
ds_dB = [-16, -0.5, +0.7,    0, +0.5,  +3.5,   +14];
[ds_B, ds_A] = invfreqz(arrayfun(@db2mag, ds_dB), (ds_f.*((2*pi)/Fs)), 6, 4);
% Stabilize polynoms
ds_Bs = polystab(ds_B) * norm(ds_B) / norm(polystab(ds_B));
ds_As = polystab(ds_A) * norm(ds_A) / norm(polystab(ds_A));
ds_H = freqz(ds_Bs, ds_As, iec_f, Fs);

% Equalizer filter, i.e. inverse from estimated transfer filter
% Swap A and B coefficients, and normalize to ds_B(1) 
eq_B = ds_As./ds_Bs(1)
eq_A = ds_Bs./ds_Bs(1)
eq_H = freqz(eq_B, eq_A, iec_f, Fs);
% Check for poles ouside unit circle
roots(eq_B)
roots(eq_A)

clf;
hold on;
semilogx(iec_f, ds_e_dB, 'g;ICS-43432 Datasheet plot (estimated);');
semilogx(iec_f, 20 * log10 (abs (ds_H)), 'r;IIR filter frequency response;');
semilogx(iec_f, (ds_e_dB + 20*log10(abs(eq_H))), 'b;Adjusted frequency response;', 'linewidth', 3);
title("ICS-43432 Adjusted frequency response");
grid minor;
xlabel('Frequency (Hz)');
xlim([10, 20000]);
ylabel('Amplitude (dB)');
ylim([-20, 20]);
legend ('boxoff');
legend ('location', 'northwest');
ylim ([-20, 20]);
grid on;
hold off;

[sos, gain] = tf2sos(eq_B, eq_A)
