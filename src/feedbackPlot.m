function feedbackPlot(WPinv, WSinv, WQinv, T, t, y, u)
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
%   --- plot of the nominal tuned system -- used in Feedback design section in the presentation 
%   --- plot of the uncertain system using the nominal tuning parameters 
%     

% 1st constraint approximation plot 
approximation1st = figure;
bodemag(WPinv, 'b');
hold on 
grid on 
grid minor 
bodemag(WSinv, 'r');
legend('1/WP', '2nd order sensitivity func.', 'location', 'southeast')
title('2nd order sensitivity approximation')
saveas(approximation1st, 'figure\approximation1st', 'epsc');

% 2nd constraint approximation plot 
approximation2nd = figure;
bodemag(WQinv, 'b'); 
grid on 
grid minor 
legend('1/WQ', 'location', 'northeast')
title('Control performace weight')
saveas(approximation2nd, 'figure\approximation2nd', 'epsc');

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

end 