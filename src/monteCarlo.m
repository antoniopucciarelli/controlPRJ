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
%   --- simulates a Monte-Carlo analysis of the system 
%

% stability derivatives
Y_v = -0.264; % 1/s      
Y_p = 0;      % m/s rad 
L_v = -7.349; % rad s/m  
L_p = 0;      % 1/s 

% control derivatives
Y_d = 9.568;    % m/s2    
L_d = 1079.339; % rad/s2  

% covariance
sigmaYv = ((4.837/100)*abs(Y_v))^2; 
sigmaLv = ((4.927/100)*abs(L_v))^2;
sigmaYd = ((4.647/100)*abs(Y_d))^2;
sigmaLd = ((2.762/100)*abs(L_d))^2;

% gaussian distribution
Y_v_gd = gmdistribution(Y_v, sigmaYv);
L_v_gd = gmdistribution(L_v, sigmaLv);
Y_d_gd = gmdistribution(Y_d, sigmaYd);
L_d_gd = gmdistribution(L_d, sigmaLd);

Y_v_all = [];
L_v_all = [];
Y_d_all = [];
L_d_all = [];

% no. of simulations
N = 500; 

for n = 1:N
    g   = 9.81;
    Y_p = 0;
    L_p = 0;

    % random function is used to take a random point for analysis
    Y_v_v(n) = random(Y_v_gd);
    L_v_v(n) = random(L_v_gd);
    Y_d_v(n) = random(Y_d_gd);
    L_d_v(n) = random(L_d_gd);
    
    % Updated non deterministic Matrixes
    Au = [Y_v_v(n), Y_p, g;
          L_v_v(n), L_p, 0;
          0,        1,   0];

    Bu = [Y_d_v(n), L_d_v(n), 0]';

    Cu = [0, 1, 0;
          0, 0, 1];

    Du = [0, 0]';

    G = ss(Au, Bu, Cu, Du);

    % connections
    G.u = '\delta_{lat}'; 
    G.y = {'p','\phi'};

    % R input/output setup 
    Rp.u = 'e_{p}'; 
    Rp.y = '\delta_{lat}';

    eInner = sumblk('e_{p} = p0 - p');

    % Rphi input/output setup 
    Rphi.u = 'e_{\phi}'; 
    Rphi.y = 'p0';

    % blocks connection
    L = connect(G_nom, Rp, eInner, Rphi, {'e_{\phi}'},{'\phi'});

    [Gm,Pm] = margin(L); % nominal margins
    Gm_n    = Gm;
    Pm_n    = Pm;

    % uncertain loop TF
    L_unc = connect(G, Rp, eInner, Rphi, {'e_{\phi}'},{'\phi'});
    Larray(:, :, n, 1) = L_unc; % Collecting Loop TF

    % computing margins for nondeterministic loop tf
    [Gm,Pm]    = margin(L_unc);
    Gm_vect(n) = Gm;            % gain margins
    Pm_vect(n) = Pm;            % phase margins
end

% figures
figure;
hist(Pm_vect,100), grid, title('Phase margin')

figure;
hist(Gm_vect,100), grid, title('Gain margin')

% parameter ditribution
figure;
hist(Y_v_v,100), grid, title('Y_v')

figure;
hist(L_v_v,100), grid, title('L_v')

figure;
hist(Y_d_v,100), grid, title('Y_d')

figure;
hist(L_d_v,100), grid, title('L_d')