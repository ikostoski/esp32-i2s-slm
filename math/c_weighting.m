%
% C-weighting IIR filter design
%
% (c)2019 Ivan Kostoski
%

clear;
format long;
pi = 3.14159265358979;

% Sampling Rate
Fs = 48000;

% IEC specified frequencies
IEC_f = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, ...
         100, 125, 160, 200, 250, 315, 400, 500, 630, 800, ...
         1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
         10000, 12500, 16000, 20000];

% Logarithmic frequencies for graphs         
f = logspace(0, 5, 2048);
         
% IEC specified attenuation for C-weighting
IEC_f_cw = [-14.3, -11.2, -8.5, -6.2, -4.4, -3.0, -2.0, -1.3, -.8, -.5, ...
            -.3, -.2, -.1, 0, 0, 0, 0, 0, 0, 0, ...
            0, 0, -.1, -.2, -.3, -.5, -.8, -1.3, -2.0, -3.0, ...
            -4.4, -6.2, -8.5, -11.2];
                 
% Analog C-weighting filter according to IEC/CD 1672.
f1 = 20.598997; %Hz 
f2 = 107.65265; %Hz
f3 = 737.86223; %Hz 
f4 = 12194.217; %Hz
C1000 = 0.0619;
NUM = [(2*pi*f4)^2*(10^(C1000/20)), 0, 0];
DEN = conv([1, +4*pi*f4, (2*pi*f4)^2], [1, +4*pi*f1, (2*pi*f1)^2]); 
% Analog transfer function
H_a = freqs(NUM, DEN, f.*(2*pi));

% Bilinear transformation of analog design to get the digital 
[B_bt, A_bt] = bilinear(NUM, DEN, 1/Fs);
H_bt = freqz(B_bt, A_bt, f, Fs);

% We will take the only the low frequency part of bilinear transformation
% Which was calculated to be approximately (the first SOS):
B_lo = [1, -2, 1];
A_lo = [1, -1.9946144559930206, 0.9946217070140836];
[sos_lo, gain_lo] = tf2sos(B_lo, A_lo);

%
% Curve fitting of the 'high' frequency part with invfreqz
%
f_hi =  [1000,  1250,  10000, 16000, 20000, 24000];
dB_hi = [   0,     0,   -4.4,  -8.6, -11.3, -15.0];
w_hi = f_hi.*((2*pi)/Fs);
mag_hi = arrayfun(@db2mag, dB_hi);
[B_hi, A_hi] = invfreqz(mag_hi, w_hi, 4, 4);
[sos_hi, gain_hi] = tf2sos(B_hi, A_hi);

% Merge SOS sections
sos = [sos_lo; sos_hi]
% Multiply the gain and reduce it a bit
gain = gain_lo * gain_hi * 0.996

% Convert to transfer function and try to stabilize 
[B_tf, A_tf] = sos2tf(sos, gain);
B_tf_s = polystab(B_tf) * norm(B_tf) / norm(polystab(B_tf));
A_tf_s = polystab(A_tf) * norm(A_tf) / norm(polystab(A_tf));
A_tf_s0 = A_tf_s(1);
% Caluclate and display stable coefficients
B = B_tf_s./A_tf_s0
A = A_tf_s./A_tf_s0
% Display the roots
roots(B)
roots(A)
H = freqz(B, A, f, Fs);

% Frequency response graph
clf;
hold on;
grid on;
xlim([10, Fs/2]);
ylim([-15, 5]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
semilogx(f, 20*log10(abs(H_a)), 'g;IEC specified transfer function for C-weighting;');
semilogx(f, 20*log10(abs(H_bt)), 'r;Bilinear trasformation;');
semilogx(f, 20*log10(abs(H)), 'b;High frequency part curve fitting with invfreqz;');
hold off

% Filter error graph
clf;
hold on;
grid on;
grid minor;
xlim([10, 20000]);
ylim([-1, +1]);
xlabel('Frequency (Hz)');
ylabel('Filter error (dB)');
semilogx(f, 20*log10(abs(H_bt)) - 20*log10(abs(H_a)), 'r;Error with just bilinear trasformation filter;');
semilogx(f, 20*log10(abs(H)) - 20*log10(abs(H_a)), 'b;Error with additional curve fitting via invfreqz;');
hold off;

% Display the sections
[sos1, gain1] = tf2sos(B, A)
