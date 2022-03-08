close all
clear 
clc

%% Quadrotor plant setup

Y_v = ureal('Y_v', -0.264, 'percentage', 4.837*3);      %uncertainty 4.837%
Y_p = 0;
L_v = ureal('L_v', -7.349, 'percentage', 4.927*3);      %uncertainty 4.927%
L_p = 0;
Y_d = ureal('Y_d', 9.568, 'percentage', 4.647*3);       %uncertainty 4.647%
L_d = ureal('L_d', 1079.339, 'percentage', 2.762*3);    %uncertainty 2.762%
g   = 9.81;

A = [ Y_v, Y_p, g;
    L_v, L_p, 0;
    0, 1, 0];
B = [Y_d, L_d, 0]';

C = [0, 1, 0;
    0, 0, 1];

D = [0, 0]';

G = ss(A,B,C,D);                                %create state-space model
G.u = '\delta_{lat}'; G.y = {'p','\phi'};       %input/output of plant
G_nom = G.nominal;                              %nominal plant model 
%% PID controller setup - roll rate

R_p = tunablePID('R_p', 'PID');                 %create PID controller
R_p.Kp.Value = 100;                     %initialize tunable PID variables
R_p.Ki.Value = 100;
R_p.Kd.Value = 100;
R_p.Tf.Value = 0.001;
R_p.u = 'e_{p}'; R_p.y = '\delta_{lat}';        %input/output of PID
Sum_ep = sumblk('e_{p} = p0 - p');           %create summing junction

%% P controller setup - roll angle

R_phi    = tunablePID('R_phi', 'P');            %create P controller
R_phi.Kp.Value = 100;                       %initialize tunable P variable
R_phi.u = 'e_{\phi}'; R_phi.y = 'p0';           %input/output of P
Sum_ephi = sumblk('e_{\phi} = \phi0 - \phi');  %create summing junction


%% Create complementary sensitivity function

F = connect(G_nom, R_p, Sum_ep, R_phi, Sum_ephi,{'\phi0'}, {'p','\phi'});
T0 = connect(G_nom, R_p, Sum_ep, R_phi, Sum_ephi,{'\phi0'}, {'e_{\phi}','\delta_{lat}'});

%% Shape function - 2nd order response

damp = 0.9;
omega_n = 10;

%ideal complementary sensitivity?????
WFinv = tf(omega_n^2,[1, 2*damp*omega_n, omega_n^2]);
% figure; bodemag(WFinv); grid on; title('Ideal compl. sensitivity FUNCTION? bode diagram');

%ideal sensitivity ???
WSinv_ideal = minreal((1-WFinv)); %zero pole cancellation and  
% figure; bodemag(WSinv_ideal); grid on; title('Ideal sensitivity FUNCTION? bode diagram');

%% Sensitivity weight Wp

% 1/Wp =( s + omega_b * A )/( s/M + omega_b ) where omega_b lowerbound of
% bandwith sensitivity, M >= 1/damping ratio and A is the low frequency
% value of  the sensitivity
omega_b = 5; A = 0.0001; M = 1.2; %can change them if you want
 WPinv = tf([1, omega_b*A],[1/M, omega_b]);
%WPinv = tf([1 5.5*A],[1/M 5.5]);

%% Control moderation

% control effort moderation weight function, achieved using an iterative approach
WQinv = 0.5 * tf([1/900 1],[1/170 1]); 

%% System constraint

rng('default');
N_TESTS = 10;
Req = [
     TuningGoal.WeightedGain('\phi0','e_{\phi}', 1/WPinv,1); 
     TuningGoal.WeightedGain('\phi0', '\delta_{lat}', 1/WQinv,1);
     ];

opt = systuneOptions('RandomStart', N_TESTS,'SoftTol',1e-7,'Display','iter');
[T, J, ~] = systune(T0, Req, opt);

R_p   = T.blocks.R_p;
R_phi = T.blocks.R_phi;

%% Plot sensitivity weight and optimized

figure;
bodemag(WPinv);
hold on;
% bodemag(WSinv_ideal);
grid on;
 hold on;
 bodemag(T(1)); % T(1) is the S in step tracking, so this is the S
legend('Wp', 'Optimized system','location','northwest');

%% Check nominal performance
figure;
bodemag(minreal(T(1)/WPinv));
grid on;
title('Sensitivity under the weight');

%% Check control sensitivity weight and optimized function
figure;
bodemag(WQinv);
grid on;
 hold on;
 bodemag(T(2)); 
legend('Wq','Optimized system');

%% Doublet response

%doublet input
dt = 0.01;
t1 = [0:dt:1]';
t2 = [1 + dt:dt:3]';
t3 = [3 + dt:dt:5]';
t4 = [5 + dt:dt:10]';
t  = [t1; t2; t3; t4];

% doublet definition
u = [ zeros(length(t1),1); 10*ones(length(t2),1); -10*ones(length(t3),1); zeros(length(t4),1)];
[y,x] = lsim(T(2).A, T(2).B, T(2).C, T(2).D, u, t);
figure;
plot(t, y, t, u);
title('Doublet response');
grid on;

%% Create uncertain model

T0 = connect(G, R_p, Sum_ep, R_phi, Sum_ephi,{'\phi0'}, {'e_{\phi}','\delta_{lat}'});
