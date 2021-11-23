%{
 **************************************************************************
 * @file    bmp2_uart_example.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.2
 * @date    19-Jun-2020
 * @brief   Simple MATLAB serial port client example for BMP2xx sensor.
 **************************************************************************
%}

%% Serial port set up
if ~exist('huart', 'var')
    huart = serial('COM3','BaudRate',9600,'Terminator','LF', 'Timeout', 10);
    fopen(huart);
end

%% Characteristic plot
N = 1000000;  % number of data points
h = figure();
subplot(2,1,1)
    pTemp = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Time [s]');
    ylabel('Temperature [degC]');
    hold on; grid on;
subplot(2,1,2)
    pDuty = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Time [s]');
    ylabel('Duty [%]');
    hold on; grid on;
    
%% Perform experiment    
k = 1;
t = 0;
ts = 0.3;

while 1
    
    rawData = fgetl(huart);
    
    if ~isempty(rawData)
        data = jsondecode(rawData);
        if isfield(data,'Temp')
            updateplot(pTemp, data.Temp, t, k, N);
        end
        if isfield(data,'Duty')
            updateplot(pDuty, data.Duty, t, k, N);
        end
        drawnow;
    end
    
    t = t + ts;
    k = k + 1;
    
    if ~isempty(h.CurrentCharacter)
        break;
    end
end

%% Close serial port and remove handler
fclose(huart);
delete(huart);
clearvars('huart');

%% Plot update function: save only last N samples
function updateplot(hplot, y, x, k, N)
    if k > N
        hplot.YData = circshift(hplot.YData, -1); 
        hplot.YData(end) = y; 
        hplot.XData = circshift(hplot.XData, -1); 
        hplot.XData(end) = x; 
    else
        hplot.YData(k) = y; 
        hplot.XData(k) = x; 
    end
end
