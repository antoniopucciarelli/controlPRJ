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
%   --- feedback design, analysis and verification for the unstable lateral
%   dynamics analised in the Gstudy.m script
%

clear variables
close all 
clc

%% lateral dynamics assembly process 
run lateralDynamics;

%% PID based loop (inner loop) assembly --> input ep = eInner
% PID controller definition
Rp = tunablePID('Rp', 'PID');
Rp.Kp.Value = 1e+2;
Rp.Ki.Value = 1e+2;
Rp.Kd.Value = 1e+2;
Rp.Tf.Value = 1e-3;

% input and output name declaration
Rp.u = 'e_{p}'; 
Rp.y = '\delta_{lat}';

% error at the inner node --> summation node
eInner = sumblk('e_{p} = p0 - p');

%% P based loop (outer loop) assembly --> input ephi = eOuter
% P controller definition 
Rphi = tunablePID('Rphi', 'P');
Rphi.Kp.Value = 1e+2;

% input and output name declaration
Rphi.u = 'e_{\phi}'; 
Rphi.y = 'p0';

% error at the outer node --> summation node
eOuter = sumblk('e_{\phi} = \phi0 - \phi');

%% complementary sensitivity assembly 
% 
%                   eOuter             eInner      delta_lat
%         phi0 --->O----->[ Rphi ]-->O------->[ Rp ]---[ G ]-+--+---> phi
%                - |               - |                       |  |
%                  |                 +<----------------------+  |
%                  +<-------------------------------------------+
%

F = connect(G, Rp, eInner, Rphi, eOuter, {'\phi_0'}, {'p', '\phi'});

%% control effort function assembly 
% there are requirements on the effort to be made in order to control the
% system 
% this control effort is related to delta_lat with respect to phi0 changes
% 
%                   eOuter             eInner      delta_lat
%         phi0 --->O----->[ Rphi ]-->O------->[ Rp ]---[ G ]-+--+---> phi
%                - |               - |                       |  |
%                  |                 +<----------------------+  |
%                  +<-------------------------------------------+
%

T0 = connect(G, Rp, eInner, Rphi, eOuter, {'\phi0'}, {'e_{\phi}', '\delta_{lat}'}); 

%% 2nd order phi0 response transfer function assembly 
% design requirements: [A] nominal performance --> phi response to phi0
% input has to follow 2nd order response with xi and natural frequency
% given 

damp    = 0.9; % 2nd order function damping coefficient
omega_n = 10;  % 2nd order function natural frequency 

% ideal complementary sensitivity function assembly 
% transfer function assembly 
WFinv = tf(omega_n^2, [1, 2*damp*omega_n, omega_n^2]);

% ideal sensitivity 
WSinv_ideal = minreal(1-WFinv); % pole-zero cancellation function 

% sensitivity weight
A = 1e-4; 
M = 1.2; 
omega_b = 5;

% 
WPinv = tf([1, omega_b*A], [1/M, omega_b]);

%% control moderation 
% this part allows to tune the control system in oder to satisfy the assignment requirements
WQinv = 0.5 * tf([1/900, 1], [1/170, 1]);

%% robust control analysis 
% enabling random number generator in the system 
rng('default');

% setting up number of tests to be done 
nTest = 10; 

% setting up system requirements -- given by assignment 
req = [ TuningGoal.WeightedGain('\phi0', 'e_{\phi}', 1/WPinv, 1);
        TuningGoal.WeightedGain('\phi0', 'delta_{lat}', 1/WQinv, 1)];

        
