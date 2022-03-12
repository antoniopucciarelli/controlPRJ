%   AEROSPACE CONTROL SYSTEM PROJECT -- AY 2021/2022
%   authors:
%       * Mate-Erik Moni 
%       * Antonio Pucciarelli 
%       * Atefeh Esmaelizadeh Rostam
%   
%   problem description: 
%   --- analysis, design and verification of a quadrirotor, ANT-R, single axis attitude control system 
%
%   this program:
%   --- studies the nominal and uncertain behaviour of the lateral dynamics
%   system (given by the assignment text)
%

clear variables
close all 
clc

%% lateral dynamics assembly process 
run lateralDynamics;

%% nominal G analysis
% poles and zeros
nominalPolesZeros = figure;
hNominal = pzplot(G.Nominal);
grid on
grid minor 

saveas(nominalPolesZeros, 'figure\nominalPolesZeros', 'epsc');

%% uncertain G analsys
% poles and zeros 
uncertainPolesZeros = figure;
hUncertain = pzplot(G);
grid on 
grid minor

saveas(uncertainPolesZeros, 'figure\uncertainPolesZeros', 'epsc');

%% frequency response study 
% bode plot 
bodePlot = figure;
bodemag(G, 'y', G.Nominal, 'k', {1e-3,1e+3});
grid on 
grid minor

saveas(bodePlot, 'figure\bodePlot', 'epsc');

%% step response study 
% this study is made varying the uncertainty in the G(s) function 
n        = 50; % number of G(s) to take into consideration  
interval = 3;  % step reponse time interval -> for the plot

% step response
stepResponse = figure;
step(usample(G,n), interval);
grid on 
grid minor 

saveas(stepResponse, 'figure\stepResponse', 'epsc');
