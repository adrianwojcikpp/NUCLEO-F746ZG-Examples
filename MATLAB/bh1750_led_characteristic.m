%{
 **************************************************************************
 * @file    bh1750_led_characteristic.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    10-Nov-2020
 * @brief   Simple MATLAB serial port client example
 **************************************************************************
%}

if ~exist('huart', 'var')
    huart = serial('COM3','BaudRate',115200,'Terminator','LF', 'Timeout', 10);
    fopen(huart);
end

duty_ref = 0 : 1 : 100;

N = length(duty_ref);

hFig = figure();
    hPlot = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Duty [%]');
    ylabel('Light intensity [lx]');
    hold on; grid on;
    
k = 1;
t = 0;
ts = 0.5;

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
    
    pause(ts);
    
    t = t + ts;
end

fclose(huart);
delete(huart);
clearvars('huart');
