%   AEROSPACE CONTROL SYSTEM PROJECT -- AY 2021/2022
%   authors:
%       * Mate-Erik Moni 
%       * Antonio Pucciarelli 
%       * Atefeh Esmaelizadeh Rostam
%   
%   problem description: 
%   --- analysis, design and verification of a quadrirotor, ANT-R, single axis attitude control system 
%   

figure 
bode(F_smpl_s); 
grid on;
title('Bode plot of uncertain Complementary sensitivity')

figure 
pzplot(F_smpl_s); 
grid on;
title('Ploes and Zeros of uncertain Complementary sensitivity')

figure 
nyquist(F_smpl_s); 
grid on;
title('Nyquist plot of uncertain Complementary sensitivity')

figure 
bode(L_smpl_s);
grid on;
title('Bode plot of uncertain Loop TF ')

figure 
pzplot(L_smpl_s);
grid on;
title('Ploes and Zeros of uncertain Loop TF ')

figure 
nyquist(L_smpl_s); 
grid on;
title('Nyquist plot of uncertain Loop TF ')

figure 
bode(L_in_smpl_s);
grid on;
title('Bode plot of uncertain Inner Loop TF ')

figure 
pzplot(L_in_smpl_s);
grid on;
title('Ploes and Zeros of uncertain Inner Loop TF ')

figure 
nyquist(L_in_smpl_s); 
grid on;
title('Nyquist plot of uncertain Inner Loop TF ')

figure 
step(F_smpl_s,1);
grid on;
title('Step response')

%% step response
figure 
step(F_smpl_s(2),1);
title('Step response of the uncertain system');
hold on
step(WFinv);        
grid on;
legend('System step response', 'Ideal')

%% Uncertain Weight
figure,bodemag(Rel_e,Info.W1,'g');
legend('Relative Error', 'Weight')
% relative error between nominal and uncertain plant fot the second chanel
% and the weight building automatically by matlab
grid on;
title('Relative error of \delta_{lat} - \phi transfer function');

%% Robust stability M-delta
figure,bodemag(M); % if Magnitude of M is less than 1 then rubust stability is satisfied
grid on;
title('M magnitude');

%% Robust Performance M-delta
figure
bodemag(S_smpl_s(1));
grid on;
hold on;
bodemag(WPinv);
legend('Sensitivity','Wp');

figure;
omega = logspace(-3,2,500);
semilogx(omega, P_inx);
grid on;
xlabel('Frequency');
ylabel('Magnitude');
title('Robust performance');