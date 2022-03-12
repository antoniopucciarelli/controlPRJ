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
%   --- plot of the nominal tuned system 
%   --- plot of the uncertain system using the nominal tuning parameters 
%     

% 1st constraint related plot 
sensitivityWeight = figure;
bodemag(WPinv);
hold on
grid on
grid minor 
bodemag(T(1));  
legend('W_{P}', 'Optimized system', 'location', 'northwest')
title('1st constraint check')
saveas(sensitivityWeight, 'figure\sensitivityWeight', 'epsc');

sensitivityCheck = figure;
bodemag(minreal(T(1)/WPinv));
grid on
grid minor 
title('Sensitivity under the weight')
saveas(sensitivityCheck, 'figure\sensitivityCheck', 'epsc');

% 2nd constraint related plot 
controlWeight = figure;
bodemag(WQinv);
grid on 
grid minor 
hold on 
bodemag(T(2));
legend('W_Q', 'Optimized system', 'location', 'southeast')
title('2nd constraint check')
saveas(controlWeight, 'figure\controlWeight', 'epsc');

% doublet response
doubletResponse = figure;
plot(t, y);
hold on 
plot(t, u);
grid on 
grid minor 
title('Doublet response')
legend('Output', 'Input');
saveas(doubletResponse, 'figure\doubletResponse', 'epsc');
