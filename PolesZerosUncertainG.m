clear all;
close all;
clc

%valori nominali
Y_v = ureal('Y_v', -0.264, 'Perc', 3*4.837);
Y_p = 0;
L_v = ureal('L_v', -7.349, 'Perc', 3*4.927);
L_p = 0;
Y_d = ureal('Y_d', 9.568, 'Perc', 3*4.647);
L_d = ureal('L_d', 1079.339, 'Perc', 3*2.762);
g   = 9.81;

% definition of the dynamic system

A = [ Y_v, Y_p, g;
    L_v, L_p, 0;
    0, 1, 0];

B = [Y_d, L_d, 0]';

C = [0, 1, 0;
    0, 0, 1];

D = [0, 0]';

%poles and zeros of the system

%G = C*(inv(s*eye(3)-A))*B +D;
G_ss = ss(A,B,C,D);
%nominal value in zero, poles gain form
figure;
pzplot(G_ss.Nominal);
grid on;

%POLES AND ZEROS OF THE UNCERTAIN SYSTEM
figure;
pzplot(G_ss);
grid on;
% % plot zeros and frequency response function
% bodemag(G_ss, G_ss.Nominal,{1e-1,1e4});
% disp('Poles');
% pole(G_ss.Nominal)
% disp('Zeros');
% tzero(G_ss.Nominal)
% grid on;

%UNCERTAINTY MODEL
%sampling value of G to store its true value
G_array = usample(G_ss, 20);

% u(1<=t<=3) = 10;
% u(3<t<=5) = -10;
T=2;
figure;
 step(G_array,T);
grid on;
