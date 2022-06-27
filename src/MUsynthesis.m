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
%   --- this program computes the MU analysis of the uncertain system
%

%% Mu Synthesis
% lftdata decomposes an uncertain object into a fixed certain part and a normalized uncertain part. 
%   lftdata can also partially decompose an uncertain object into an uncertain part and a normalized uncertain part. Uncertain objects (umat, ufrd, uss) are represented as certain (i.e., not-uncertain) objects in feedback with block-diagonal concatenations of uncertain elements.
% [M,Delta] = lftdata(A) separates the uncertain object A into a certain object M 
%   and a normalized uncertain matrix Delta such that A is equal to lft(Delta,M), as shown below.
[P,Delta, BlkStruct] = lftdata(G); 

P.u        = {'w1','w2','w3','w4','\delta_{lat}'}; % inputs of P
P.y        = {'z1','z2','z3','z4', 'p','\phi'};    % outputs of P
eOuter_new = sumblk('e_{\phi} = - \phi');

M_mu = connect(Rphi,eInner, Rp , P, eOuter_new,{'w1','w2','w3','w4'}, {'z1','z2','z3','z4'}); % constructing M for Mu thynthesis, with 4 inputs and 4 outputs

omega = logspace(-3, 2, 500); % frequency range

bounds     = mussv(frd(M_mu,omega), [-1 0; -1 0; -1 0; -1 0]); % real uncertanities
bounds_com = mussv(frd(M_mu,omega), [ 1 0;  1 0;  1 0;  1 0]); % complex uncertainities

figure
sigma(bounds); % singulr value computation
grid on;
grid minor; 
title('\mu synthesis');