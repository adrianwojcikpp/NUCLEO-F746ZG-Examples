%% L11 CMSIS pt. 2
%%% File info 
%
% *************************************************************************
%
%  @file     unit_test_pid.m
%  @author   Adrian Wojcik
%  @version  1.0
%  @date     17-Dec-2019 10:33:27
%  @brief    Simple PID controller unity test
%
% *************************************************************************
%
close all; clc;
clear all
%% Test signal
% sample time & freq
ts = 0.001; % [s]
fs = 1/ts;  % [Hz]

% test time
tmax = 1;   % [s]

% time vector
tvec = (0 : ts : tmax-ts)';

% sample vector
nvec = 0 : length(tvec)-1;

% test signal - square wave
T = 0.1; % [s] signal period
refvec = single(zeros(size(tvec)));
for i = nvec+1
    if mod(i*ts,T) < T/2
       refvec(i) = 3.3;
    else
       refvec(i) = 0.0; 
    end%if
end%for

%% Test plant model

% -- GAIN [-] -------------------------------------------------------------
K = 10.0;
    
% -- TIME CONSTANT [s] ----------------------------------------------------
T = 0.5; 
    
% -- S-OPERATOR [1/s] -----------------------------------------------------
s = tf('s');

% -- PLANT TRANSFER FUNCTION ----------------------------------------------
G = K / (1 + T*s);

%% PID controller tunning
% simple pole-placement example

% -- REFERENCE CLOSED LOOP TIME CONSTANT [s] ------------------------------
Tref = 0.01; 

% -- PID CONTROLER GAINS --------------------------------------------------
Ki = 1 / ( K * Tref );
Kp = Ki * T;
Kd = 0;

% -- PID CONTROLLER TRANSFER FUNCTION --------------------------------------
H = Kp + Ki/s + Kd*s;

% -- CLOSED-LOOP SYSTEM ---------------------------------------------------
S_out = feedback(H*G, 1);
S_pid = feedback(H, G);

% -- CLOSED-LOOP SYSTEM RESPONSE COMPUTING --------------------------------
outvec = lsim(S_out,refvec,tvec);
controlvec = lsim(S_pid,refvec,tvec);
errvec = refvec - outvec;

pid_out_v1 = controlvec;

%% FOR-LOOP REFERENCE PID CONTROLLER RESPONSE COMPUTING

x = single(errvec); % error
xi = single(0);  % error integral
y1 = single(zeros(size(errvec)));
y2 = single(zeros(size(errvec)));

    y3 = single(zeros(size(errvec)));

% -- PARALLEL FORM OF CONTINUOUS-TIME PID CONTROLLER --> CMSIS FORMULA ----
Kp2 = single(Kp);
Ki2 = single(Ki * ts); 
Kd2 = single(Kd / ts);

% -- CMSIS ALGORITHM GAINS FORMULAS ---------------------------------------
A0 = Kp2 + Ki2 + Kd2;
A1 = (-Kp2 ) - (2 * Kd2 );
A2 = Kd2;

% CMSIS FORM                                      % n = 1
y2(1) = A0 * x(1);
y3(1) = A0 * x(1);

% EXPLICIT FORM
xi = xi + x(1);
xd = single(0);  % error derivative
y1(1) = Kp2*x(1) + Ki2*xi + Kd2*xd;

% CMSIS FORM                                      % n = 2
y2(2) = A0 * x(2) + A1 * x(1) + y2(1); % code order
y3(2) = y2(1) + A0 * x(2) + A1 * x(1); % doc order

% EXPLICIT FORM
xi = xi + x(2);
xd = x(2) - x(1);   
y1(2) = Kp2*x(2) + Ki2*xi + Kd2*xd;

for n = 3 : length(tvec)                          % n >= 3
    
    % EXPLICIT FORM
    xi = xi + x(n);
    xd = x(n) - x(n-1);   
    y1(n) = Kp2*x(n) + Ki2*xi + Kd2*xd;

    % CMSIS FORM
    y2(n) = A0 * x(n) + A1 * x(n-1) + A2 * x(n-2) + y2(n-1); % code order
    y3(n) = y3(n-1) + A0 * x(n) + A1 * x(n-1) + A2 * x(n-2); % doc order
    
end

pid_in = x;
pid_out_v21 = y1;
pid_out_v22 = y2;

%% RESULT PLOT
OFFSET = 0;

hold on;
    stairs(tvec, refvec); 
    stairs(tvec, pid_out_v1  + OFFSET);
    stairs(tvec, pid_out_v21 + OFFSET);
    stairs(tvec, pid_out_v22 + OFFSET);
grid on;
hold off;
xlabel('Time [s]');
legend('REF', 'OUTPUT (lsim)', 'OUTPUT (for loop EXPLICIT)', ...
                               'OUTPUT (for loop CMSIS)');

%% SAVE TEST DATA TO .CSV
VEC2CSV('pid_x.csv', pid_in);
VEC2CSV('pid_y.csv', zeros(size(pid_in)));
VEC2CSV('pid_yref.csv', pid_out_v22);   

VEC2CSV('pid_gains.csv', [Kp2 Ki2 Kd2]);