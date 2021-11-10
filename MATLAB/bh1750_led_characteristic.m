%{
 **************************************************************************
 * @file    bh1750_led_characteristic.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    10-Nov-2020
 * @brief   Simple MATLAB serial port client example
 **************************************************************************
%}

%% Serial port set up
if ~exist('huart', 'var')
    huart = serial('COM3','BaudRate',115200,'Terminator','LF', 'Timeout', 10);
    fopen(huart);
end

%% Reference control signal
duty_ref = 0 : 1 : 100;
N = length(duty_ref);

%% Characteristic plot
hFig = figure();
    hPlot = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Duty [%]');
    ylabel('Light intensity [lx]');
    hold on; grid on;
 
%% Log file
filename = [ 'BH1750_LED_' datestr(datetime, 30)];

%% Perform experiment
k = 1;               % [-]
t = 0;               % [s]
ts = 1.0;            % [s]
respone_delay = 0.2; % [s]

for i = 1: length(duty_ref)
    
    str = sprintf("%3d", duty_ref(i));
    fprintf(huart, str);
    
    rawData = fgetl(huart);
    
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
duty = hPlot.XData;
light = hPlot.YData;
save([filename '.mat'], 'duty', 'light');
savefig(hFig, [filename '.fig']);

%% Close serial port and remove handler
fclose(huart);
delete(huart);
clearvars('huart');
