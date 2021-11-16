function angle = lamp_brightness2angle(brightness)
% LAMP_BRIGHTNESS2ANGLE Computes firing angle for TRIAC in incandescent 
%   light bulb control circuit for given brightness expressed in percent.
%   Solution is based on numerical solution of ciruit algebraic equation.
%
%   ANGLE = LAMP_BRIGHTNESS2ANGLE(BRIGHTNESS) return vector with TRIAC 
%   firing angles in radians for given brightness expressed in percent. 
%   Result vector is the size as input vector.
%
%   See also VPASOLVE.

%% Brightness normalization
b = brightness / 100; % <0, 100> -> <0,1>
for i = 1 : length(b)
    b(i) = max([min([b(i), 1]), 0]); % saturation <0,1>
end

%% Power-brightness relationship
p = sqrt(b);  % b = p^2

%% Numerical solution of ciruit algebraic equation
syms angle_sym;
% result vector
angle = nan(size(p)); % [rad] 

for i = 1 : length(b)
    % Relationship between TRIAC firing angle and relative power for 
    % ohmic resistor:
    %                                               
    % p(alpha) = (umax^2/R) * int_{alpha}^{pi}( sin^2(theta) dtheta )
    %
    angle_tmp = vpasolve( 2*pi*(1-p(i)) == 2*angle_sym - sin(2*angle_sym) ); 
    angle(i) = double(angle_tmp);
end