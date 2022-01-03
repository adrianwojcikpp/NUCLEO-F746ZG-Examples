%% L10 CMSIS pt. 1
%%% File info 
%
% ************************************************************************
%
%  @file     unit_test_fir.m
%  @author   Adrian Wojcik
%  @version  1.0
%  @date     12-Dec-2020 19:59:05
%  @brief    Simple digital low-pass filter design and test
%
% ************************************************************************
%
%% Test signal
% sample time & freq
ts = 0.001; % [s]
fs = 1/ts;  % [Hz]

% test time
tmax = 1;   % [s]

% time vector
tvec = 0 : ts : tmax-ts;

% sample vector
nvec = 0 : length(tvec)-1;

% test signal - square wave
T = 0.1; % [s] signal period
xvec = zeros(size(tvec));
for i = nvec+1
    if mod(i*ts,T) < T/2
       xvec(i) = 3.3;
    else
       xvec(i) = 0.0; 
    end%if
end%for

% test signal amplitude spectrum
% frequency vector
frange = (-1/2 : 1/length(nvec) : 1/2-1/length(nvec)); % [-]
fxvec = frange*fs;                               % [Hz]
% amplitude response
Axvec = abs(fftshift(fft(xvec,length(nvec)))); % [-]
Axvec = Axvec / length(nvec);

%% FIR design

% Filter desired parameters
% cut-off frequency
f1 = 10;  % [Hz]
% frequency at the end transition band
f2 = 50; % [Hz]
% transition band length
df = f2 - f1;
% stopband attenuation of A [dB] at f2 [Hz]
A = 50; % [dB]
               
% Filter order estimation
% with 'fred harris rule of thumb'
N = round( (fs/df)*A / 22 );

% Filter object with fir1 funcion
% normalised frequency
w = f1 / (fs/2); % [-]
% low pass filter
b = fir1(N, w, 'low');

% Filter frequency response
% no. of samples
n = 10^4; % [-]
% frequency vector
frange = (-1/2 : 1/n : 1/2-1/n); % [-]
fhvec = frange*fs;               % [Hz]
% amplitude response
Ahvec_v1 = abs(freqz(b, 1, 2*pi*frange)); % [-] frequency response
Ahvec_v1 = 20*log10(Ahvec_v1); % [dB]
Ahvec_v2 = abs(fftshift(fft(b,n)));       % [-] impulse response fft 
Ahvec_v2 = 20*log10(Ahvec_v2); % [dB]

% Test signal filtration
xfvec = filter(b,1,xvec);

% filtered signal amplitude spectrum
% amplitude response
Axfvec = abs(fftshift(fft(xfvec,length(nvec)))); % [-]
Axfvec = Axfvec / length(nvec);

%% Plot results
fminmax = [-10 fs/2];
tminmax = [0 tmax];
Aminmax = [-0.1 3.5];
AdBminmax = [-120 5];

subplot(2,2,1)
  plot(nvec/fs, xvec, 'b'); grid on;
  xlabel("Time [s]");
  ylabel("Amplitude [-]");
  axis([tminmax Aminmax]);
  title('Orginal test signal time series');
  hold off;
subplot(2,2,2)
  ax = plotyy (fxvec, Axvec, ...
               fhvec, Ahvec_v1, ...
               @plot, @plot); hold on; grid on;
  xlabel("Frequency [Hz]");
  ylabel(ax(1), "Amplitude spectrum [-]");
  ylabel(ax(2), "Amplitude response [dB]"); 
  plot([f1 f1], AdBminmax, 'k--');
  plot([f2 f2], AdBminmax, 'k--'); 
  set(ax(1),'YLim', [0 0.51*3.3], 'XLim', fminmax);
  set(ax(2),'YLim', AdBminmax, 'XLim', fminmax);
  set(ax(2),'XTick',[], 'xcolor',[1 1 1]);
  title('Original test signal ampltitude spectrum');
  hold off;
subplot(2,2,3)
  plot(nvec/fs, xfvec, 'b'); grid on; hold on;
  xlabel("Time [s]");
  ylabel("Amplitude [-]");
  axis([tminmax Aminmax]);
  title('Filtered test signal time series');
  hold off;
subplot(2,2,4)
  ax = plotyy (fxvec, Axfvec, ...
               fhvec, Ahvec_v2, ...
               @plot, @plot); hold on; grid on;
  xlabel("Frequency [Hz]");
  ylabel(ax(1), "Amplitude spectrum [-]");
  ylabel(ax(2), "Amplitude response [dB]"); 
  plot([f1 f1], AdBminmax, 'k--');
  plot([f2 f2], AdBminmax, 'k--'); 
  set(ax(1),'YLim', [0 0.51*3.3], 'XLim', fminmax);
  set(ax(2),'YLim', AdBminmax, 'XLim', fminmax);
  set(ax(2),'XTick',[], 'xcolor',[1 1 1]);
  title('Filtered test signal ampltitude spectrum');
  hold off;
  
%% Save data to .csv files

% Number of taps (== number of coefficients == order+1) 
VEC2CSV('fir_ntaps.csv', N+1, 0);

% Test signal
VEC2CSV('fir_x.csv', xvec, 30);

% Reference output signal
VEC2CSV('fir_yref.csv', xfvec, 30);

% Filter coefficients
VEC2CSV('fir_b.csv', b, 30);

% Filter initial state
VEC2CSV('fir_state_init.csv', zeros(size(b)), 1);