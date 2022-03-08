% ANT-R -- data allocation -- nominal
% dynamics coefficients
% stability derivatives 
Y_v = -0.264;     % lateral acceleration vs lateral velocity 
Y_p =  0.0;       % lateral acceleration vs roll rate 
L_v = -7.349;     % roll acceleration vs lateral velocity
L_p =  0.0;       % roll acceleration vs roll rate 

% control derivatives
Y_d =  9.568;     % lateral acceleration vs command
L_d =  1079.339;  % roll acceleration vs command

% physics -- earth
g   =  9.81;      % acceleration of gravity 

save ANTRdata.mat