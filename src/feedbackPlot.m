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
bodemag(WPinv, 'b');
hold on
grid on
grid minor 
bodemag(T(1), 'r');  
legend('1/WP', 'Tuned system', 'location', 'northwest')
title('1st constraint check')
saveas(sensitivityWeight, 'figure\sensitivityWeight', 'epsc');

% sensitivity vs performance weight check
sensitivityCheck = figure;
bodemag(minreal(T(1)/WPinv), 'b');
grid on
grid minor 
title('Sensitivity under the weight: |WP * S|_{Inf} < 1')
saveas(sensitivityCheck, 'figure\sensitivityCheck', 'epsc');

% 2nd constraint related plot 
controlWeight = figure;
bodemag(WQinv, 'b')
grid on 
grid minor 
hold on 
bodemag(T(2), 'r')
legend('1/WQ', 'Tuned system', 'location', 'southeast')
title('2nd constraint check')
saveas(controlWeight, 'figure\controlWeight', 'epsc');

% doublet response
doubletResponse = figure;
plot(t, y, 'b', 'linewidth', 2);
hold on 
plot(t, u, '--r', 'linewidth', 1.5);
grid on 
grid minor 
title(['Doublet response -> Delta max = ' num2str(max(abs(y)))])
legend('Output: delta', 'Input: doublet');
saveas(doubletResponse, 'figure\doubletResponse', 'epsc');
