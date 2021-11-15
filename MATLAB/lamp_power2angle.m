function a = lamp_power2angle(d)

d = d / 100;
syms a_sym;
a = nan(size(d));
for i = 1 : length(d)
    rslt = vpasolve(2*pi*(1-sqrt(d(i))) == 2*a_sym - sin(2*a_sym)); 
    a(i) = double(rslt);
end