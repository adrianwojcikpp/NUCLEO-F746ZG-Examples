%{
 **************************************************************************
 * @file    bh1750_lamp_characteristic.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    11-Nov-2020
 * @brief   TODO
 * @note    TODO : protocol description
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
    
    str = sprintf("%3d", angle_ref(i));
    fprintf(huart, str);
    pause(respone_delay);
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
