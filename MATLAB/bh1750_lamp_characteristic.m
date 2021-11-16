%{
 **************************************************************************
 * @file    bh1750_lamp_characteristic.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    11-Nov-2020
 * @brief   Input-output characteristic of lamp dimmer with light 
 *          intensity digital sensor
 * @note    Scripts sends control commands with new TRIAC firing angle
 *          every «ts» seconds. 
 *
 *          Control command is a three-character decimal number in range 
 *          <160, 10>, terminated with line feed character ('\n'), e.g.:
 *          ' 20\n'
 *           
 *          After sending control command, script receives response within 
 *          «respone_delay» seconds. Response is a sequence of two numbers:
 *          three-character reference TRIAC firing angle in degrees and 
 *          six-character sensor measurement in lux, separated with comma 
 *          and space (', '), terminated with carriage return and line feed 
 *          characters ('\r\n'), e.g.:
 *          ' 20,  31256\r\n' 
 *          
 *          After «N» responses, script displays input-output
 *          characteristic and save figure and data to file. 
 **************************************************************************
%}

%% Serial port set up
if ~exist('huart', 'var')
    huart = serial('COM3','BaudRate',9600,'Terminator','LF', 'Timeout', 10);
    fopen(huart);
end

%% Reference control signal
angle_ref = 160 : -2 : 10;
N = length(angle_ref);

%% Characteristic plot
hFig = figure();
    hPlot = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Angle [deg]');
    ylabel('Light intensity [lx]');
    hold on; grid on;
 
%% Log file
filename = [ 'BH1750_LAMP_' datestr(datetime, 30)];

%% Perform experiment
k = 1;               % [-]
t = 0;               % [s]
ts = 2.0;            % [s]
respone_delay = 1.0; % [s]

for i = 1: length(angle_ref)
  
    rawData = [];
    while isempty(rawData)
        str = sprintf("%3d", angle_ref(i));
        fprintf(huart, str);
        pause(respone_delay);
        rawData = fgetl(huart);
    end
    
    if ~isempty(rawData)
        data = str2num(rawData);
        if length(data) == 2
            hPlot.XData(i) = data(1);
            hPlot.YData(i) = data(2);
        end
        drawnow;
    end
    pause(ts - respone_delay);    
    t = t + ts;
end

%% Save data
angle = hPlot.XData;
light = hPlot.YData;
save([filename '.mat'], 'angle', 'light');
savefig(hFig, [filename '.fig']);

%% Close serial port and remove handler
fclose(huart);
delete(huart);
clearvars('huart');

%% Compare with analytic formula
data = load([filename '.mat']);

lamp_angle2brightness = @(angle)( ((pi - angle + 0.5*sin(2*angle))/pi).^2 );

angle = deg2rad(data.angle);
light = (data.light - min(data.light))/(max(data.light) - min(data.light));

time_shift = 0.5; % [ms]
phase_shift = (time_shift / 20) * pi;

hFig = figure();
    hPlot = plot(angle, light, ...
                 angle, lamp_angle2brightness(angle + phase_shift));
    xlabel('Angle [rad]');
    ylabel('Normalized light intensity [lx]');
    legend('Experimental data', 'Analytic formula');
    hold on; grid on;

%% Verify linearization procedure
brightness = 0:1:100;
angle = lamp_brightness2angle(brightness)-phase_shift;

for i = 1 : length(angle)
    angle(i) = max([min([angle(i), pi]), 0]); % saturation <0,pi>
end

hFig = figure();
    hPlot = plot(brightness, 100*lamp_angle2brightness(angle+phase_shift));
    xlabel('Brightness [%]');
    ylabel('Brightness [%]');
    hold on; grid on;

%% Generate lookup table
csvwrite('lamp_brightness2angle_LookUpTable.csv', rad2deg(angle));