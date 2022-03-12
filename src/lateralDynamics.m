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
%   --- assembles the lateral dynamics of the ANT-R quadrirotor 
%   --- adds the data path for the quadrirotor study  
%

%% lateral dynamics system properties allocation
% data allocation -- nominal 
load('ANTRdata.mat'); 

% stability derivatives -- uncertainty expressed as 3 * sigma with Gauss distribution
Y_v = ureal('Y_v', Y_v, 'percentage', 4.837*3); % 1/s      % uncertainty 4.837%
L_v = ureal('L_v', L_v, 'percentage', 4.927*3); % rad s/m  % uncertainty 4.927%

% control derivatives -- uncertainty expressed as 3 * sigma with Gauss distribution
Y_d = ureal('Y_d', Y_d, 'percentage', 4.647*3); % m/s2    % uncertainty 4.647%
L_d = ureal('L_d', L_d, 'percentage', 2.762*3); % rad/s2  % uncertainty 2.762%

%% lateral dynamics system assembly 
%
% xdot = A * x + B * u 
% y    = C * x + D * u
%

A = [ Y_v, Y_p, g;
      L_v, L_p, 0;
        0,   1, 0];
    
B = [Y_d; 
     L_d;
       0];

C = [0, 1, 0;
     0, 0, 1];

D = [0; 
     0];

%% conversion from time domain to state space domain 
% lateral dynamics state space model assembly 
G = ss(A, B, C, D);

% input and output declaration 
% input name
G.u = '\delta_{lat}'; 
% output name
G.y = {'p','\phi'};

% setting up nominal G -> nominal plant model 
G_nom = G.nominal;
