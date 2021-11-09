%{
 **************************************************************************
 * @file    MATLAB GET Example/uart_example.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.2
 * @date    19-Jun-2020
 * @brief   Simple MATLAB serial port client example
 **************************************************************************
%}

huart = serial('COM3','BaudRate',115200,'Terminator','LF');
fopen(huart);

N = 1000000;

h = figure();
    pLight = plot(nan(N,1),nan(N,1), 'k');
    xlabel('Time [s]');
    ylabel('Light intensity [lx]');
    hold on; grid on;
    
k = 1;
t = 0;
ts = 0.3;

while 1
    
    rawData = fgetl(huart);
    
    if ~isempty(rawData)
        data = jsondecode(rawData);
        if isfield(data,'Light')
            updateplot(pLight, data.Light, t, k, N);
        end
        drawnow;
    end
    
    t = t + ts;
    k = k + 1;
    
    if ~isempty(h.CurrentCharacter)
        break;
    end
end

fclose(huart);
delete(huart);
clearvars('huart');

function updateplot(hplot, x, y, k, N)
    if k > N
        hplot.YData = circshift(hplot.YData, -1); 
        hplot.YData(end) = x; 
        hplot.XData = circshift(hplot.XData, -1); 
        hplot.XData(end) = y; 
    else
        hplot.YData(k) = x; 
        hplot.XData(k) = y; 
    end
end
