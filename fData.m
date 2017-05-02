t = time/1000;
t = t - t(1);
u = duty * 0.0075;
u = u(1:260);
y = pos/256;

% filtrer les données
%yf = smooth(t, y, 0.1, 'rloess');
yf = resample(y, t, 125);
yf = yf(1:260);

step9 = iddata(yf, u, 0.008, 'InputName', 'Voltage', 'InputUnit', 'V', 'OutputName', 'Angle', 'OutputUnit', 'deg');
