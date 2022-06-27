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
%   --- uncertain feedback design of G with respect knowing the controller 
%   --- Robust stability analysis with Monte carlo simulation and Mu 
%   --- Robust Performance

%% lateral dynamics assembly process 
%run feedbackDesign;

%% Uncertain Representation
% connections
S_unc = connect(G, Rp, eInner, Rphi, eOuter, {'\phi0'}, {'e_{\phi}', '\delta_{lat}'});  % build sensitivity TF
F_unc = connect(G, Rp, eInner, Rphi, eOuter,{'\phi0'}, {'p','\phi'});                   % build uncertain complementary sensitivity TF
L_unc = connect(G, Rp, eInner, Rphi,{'e_{\phi}'}, {'p','\phi'});                        % build uncertain loop TF

% plant from delta_lat to p
G1       = G(1);  
G1.u     = '\delta_{lat}';  
G1.y     = 'p';
L_in_unc = connect(G1, Rp, {'e_{p}'},{'p'});

% sensitivity and complementary sensitivity assembly 
S_smpl_s = minreal(tf(usample(S_unc, 50)));  % simplified sensitivity TF
F_smpl_s = minreal(tf(usample(F_unc, 50)));  % simplified complementary sensitivity TF

% check stability
ff = isstable(F_unc); % Complementary sensitivity is stable
tt = isstable(S_unc); % Sensitivity is stable
ll = isstable(L_unc); % Loop TF isn't stable

L_smpl_s    = minreal(tf(usample(L_unc, 50)));      % simplified Loop TF
L_in_smpl_s = minreal(tf(usample(L_in_unc, 50)));   % simplified inner Loop TF
pol         = pole(L_in_smpl_s);                    % poles of uncertain Loop TF
zer         = zero(L_in_smpl_s);                    % zeros of uncertain Loop TF

%% uncertain doublet response
figure
hold on

for i = 1:100
    S = usample(S_unc);
    u = [ zeros(length(t1),1); 10*ones(length(t2),1); -10*ones(length(t3),1); zeros(length(t4),1)];
    [y,x] = lsim(S(2).A, S(2).B, S(2).C, S(2).D, u, t);
    plot(t, y);
end    

title(['Doublet response -> Delta max = ' num2str(max(abs(y)))]);
grid on;

%% Uncertain Weight
G_phi = G(2,1);  % from \delta_lat to phi % take the outer loop

G_phi_nom  = getNominal(G_phi); % nominal plant
G_phi_smpl = usample(G_phi,60); % uncertain plant

[G_phi,Info] = ucover(G_phi_smpl,G_phi_nom,1); % the weight order is one
% usys = ucover(Parray,Pnom,ord) returns an uncertain model usys with nominal value Pnom and whose range of 
%   behaviors includes all responses in the LTI array Parray. The uncertain model structure is of the form usys=Pnom(I+W(s)Δ(s)), where:
% Δ is a ultidyn object that represents uncertain dynamics with unit peak gain.
% W is a stable, minimum-phase shaping filter of order ord that adjusts the amount of uncertainty at each frequency.
%    For a MIMO Pnom, W is diagonal, with the orders of the diagonal elements given by ord.
Rel_e = (G_phi_nom-G_phi_smpl)/G_phi_nom;   % relative error between nominal and uncertain plant
W_I   = minreal(tf(Info.W1));               %relative error upperbound which is the uncertian weight of the system

%% M-Delta Representation
W_I.u       = '\delta_{lat}';                                                       % weight input
W_I.y       = 'z';                                                                  % weight output
G_n         = getNominal(G);                                                        % nominal plant
G_n.u       = 'u_delta_{lat}';                                                      % input of nominal plant
Sum_u       = sumblk('u_delta_{lat} = \delta_{lat} + w');                           % multipicative sum
eOuter_new  = sumblk('e_{\phi} = - \phi');                                          % not considering setpoint in sum block
M           = connect(Rphi, eInner, Rp, W_I, Sum_u, G_n, eOuter_new,{'w'}, {'z'});  % M
M_s         = minreal(M);                                                           % simplified M TF
get(M_s);

%% Robust Performance
omega_bar   = logspace(-3,5,500);           % frequency range
Wp_all      = frd(S(1)/WPinv, omega_bar);   % computing wp for all frequencies
Wp_all      = Wp_all.ResponseData(:);
M_all       = frd(M, omega_bar);            % computing M for all frequencies
M_all       = M_all.ResponseData(:);
P_inx       = abs(M_all) + abs(Wp_all);     % Performance index for RP criteria

% plotting data 
run uncertainPlots;