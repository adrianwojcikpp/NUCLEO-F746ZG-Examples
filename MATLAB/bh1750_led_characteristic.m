%{
 **************************************************************************
 * @file    bh1750_led_characteristic.m
 * @author  AW             Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    10-Nov-2020
 * @brief   Input-output characteristic of LED controller with light 
 *          intensity digital sensor
 * @note    Scripts sends control commands with new PWM signal duty cycle
 *          every «ts» seconds. 
 *
 *          Control command is a three-character decimal number in range 
 *          <0, 100>, terminated with line feed ('\n') character, e.g.:
 *          ' 20\n'
 *           
 *          After sending control command, script receives response within 
 *          «respone_delay» seconds. Response is a sequence of two numbers:
 *          three-character reference duty cycle in percents and 
 *          six-character sensor measurement in lux, separated with comma 
 *          and space (', '), terminated with line feed character('\n'),
 *          e.g.:
 *          ' 20,  31256\n' 
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
    
    rawData = [];
    while isempty(rawData)
        str = sprintf("%3d", duty_ref(i));
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
duty = hPlot.XData;
light = hPlot.YData;
save([filename '.mat'], 'duty', 'light');
savefig(hFig, [filename '.fig']);

%% Close serial port and remove handler
fclose(huart);
delete(huart);
clearvars('huart');
