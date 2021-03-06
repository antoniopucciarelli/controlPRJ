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
%   --- feedback design for the nominal part of G with respect to given control system behaviour
%   --- analysis and verification for the unstable lateral dynamics analised in the Gstudy.m script
%       

clc

%% lateral dynamics assembly process 
run lateralDynamics;

%% PID based loop (inner loop) assembly --> input ep = eInner
% PID controller definition -- roll rate
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
% P controller definition -- roll angle
Rphi = tunablePID('Rphi', 'P');
Rphi.Kp.Value = 1e+2;

% input and output name declaration
Rphi.u = 'e_{\phi}'; 
Rphi.y = 'p0';

% error at the outer node --> summation node
eOuter = sumblk('e_{\phi} = \phi0 - \phi');

%% complementary sensitivity assembly 
% lateral dynamics block + P & PID controllers
% it allows to understand how the double closed loop system behaves with respect to phi0
% 
%             eOuter        p0   eInner       delta_lat
%   phi0 --->O----->[ Rphi ]-->O------->[ Rp ]---------[ G ]-+--+---> phi
%          - |               - |                           p |  |
%            |                 +<----------------------------+  |
%            +<-------------------------------------------------+
%

F = connect(G_nom, Rp, eInner, Rphi, eOuter, {'\phi0'}, {'p', '\phi'});

%% control effort function assembly 
% this transfer function for the double closed loop system relates the input phi0 with the 
% roll angle error and the lateral dynamics input (delta)
% 
%             eOuter        p0   eInner       delta_lat
%   phi0 --->O----->[ Rphi ]-->O------->[ Rp ]---------[ G ]-+--+---> phi
%          - |               - |                           p |  |
%            |                 +<----------------------------+  |
%            +<-------------------------------------------------+
%
% connect() function
% input: 
% --- phi0
% outputs:  
% --- phi error (eOuter)
% --- delta
CL0 = connect(G_nom, Rp, eInner, Rphi, eOuter, {'\phi0'}, {'e_{\phi}', '\delta_{lat}'}); 

%% 2nd order phi0 response transfer function assembly 
% design requirements: [A] nominal performance --> phi response to phi0
% input has to follow 2nd order response with damping (damp) and natural frequency (omega_n) given 

% setting up variables
damp    = 0.9; % 2nd order function damping coefficient
omega_n = 10;  % 2nd order function natural frequency 

% ideal performance complementary sensitivity function assembly WFinv
% WFinv relates phi0 to phi 
% y = T * r -> skogestad (2.44)  
WFinv = tf(omega_n^2, [1, 2*damp*omega_n, omega_n^2]);

% ideal performance sensitivity function assembly WSinv 
% eOuter = phi0 - phi = (1 - WFinv) * phi0 
% T0 links eOuter (e_phi) to phi0 input 
WSinv = minreal(1 - WFinv); % pole-zero cancellation after sensitivity function assembly  

% performance sensitivity weight WPinv 
% coefficients computation 
[A, M, omega_b] = sensitivityWeight(damp, omega_n, false); 

% performance transfer function assembly 
% WPinv relates phi0 to eOuter
WPinv = tf([1, omega_b*A], [1/M, omega_b]);

%% control moderation
% design requirements: [B] nominal performance --> phi response to phi0 attenuation
% for a doublet input at |phi0| = 10 -> |delta| <= 5 
% this time it has been used the same (2.44) refernce equation from skogestad book 
% but the tuning has been done throught a trial and error approach 
A       = 0.5;
M       = 0.10;
omega_b = 1.43e+3; 

% control effort moderation weight function
% WQinv relates phi0 to delta 
WQinv = tf([1, omega_b*A], [1/M, omega_b]); 

%% system constraint  
% enabling random number generator in the system  
rng('default');

% setting up number of tests to be done by the system tuning algorithm -- systune --
nTest = 10; 

% setting up system requirements -- given by assignment 
% --> TuningGoal.WeightedGain allows setting up a frequency-weighted gain
% constraint 
req = [ TuningGoal.WeightedGain('\phi0', 'e_{\phi}', 1/WPinv, 1);
        TuningGoal.WeightedGain('\phi0', '\delta_{lat}', 1/WQinv, 1) ];

%% controllers tuning -- P & PID
% setting up tuning options
% -- relative tolerance setup -> SoftTol = 1e-7
% -- # of tests to be done -> RandomStart = nTest  
opt = systuneOptions('RandomStart', nTest, 'SoftTol', 1e-7, 'Display', 'final');

% tuning control system 
% systune gets in input parametrized transfer functions and parametrized constraint/requirements 
% input definition: 
% -- closed loop model -> CL0 
% -- control constraints/goals -> req
[CL, J, ~] = systune(CL0, req, opt); % CL: sensitivity

% getting values from the tuning results
Rp   = CL.blocks.Rp;
Rphi = CL.blocks.Rphi;

% block coefficients after tuning
[Rphi_Kp,~,~,~] = piddata(Rphi);
[Rp_Kp,Rp_Ki,Rp_Kd,~] = piddata(Rp);
fprintf('\nNominal system tuning results:\nP block:\n\tKp = %f\n\n', Rphi_Kp);
fprintf('PID block:\n\tKp = %f\n\tKi = %f\n\tKd = %f\n', Rp_Kp, Rp_Ki, Rp_Kd);

% setting up input and output names 
Rp.u = 'e_{p}'; 
Rp.y = '\delta_{lat}';

% setting up input and output names
Rphi.u = 'e_{\phi}'; 
Rphi.y = 'p0';

%% doublet input response analysis
% setting up input 
dt = 1e-2;                      % computational time step 
t1 = 0:dt:1;                    % 1st time interval 
t2 = 1 + dt:dt:3;               % 2nd time interval
t3 = 3 + dt:dt:5;               % 3rd time interval 
t4 = 5 + dt:dt:10;              % 4th time interval 
t  = [t1'; t2'; t3'; t4'];      % overall time interval 

% doublet input declaration 
% t1 -> 0
% t2 -> 10 
% t3 -> -10 
% t4 -> 0
u = [ zeros(length(t1),1); 
      10*ones(length(t2),1); 
      -10*ones(length(t3),1); 
      zeros(length(t4),1) ];

% computing system response and translating it into time domain 
[y, x] = lsim(CL(2).A, CL(2).B, CL(2).C, CL(2).D, u, t);

%% figure plot
feedbackPlot(WPinv, WSinv, WQinv, CL, t, y, u);
