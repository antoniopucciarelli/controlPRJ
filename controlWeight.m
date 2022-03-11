function [gain, control1, control2] = controlWeight(args) 
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
%
%   input:
%   --- args: boolean value for the print of results
%
%   output:
%

close all 

%% input assembly 
% step input declaration and shifted in time 
step1 = tf(10, [1,0], 'InputDelay', 1);
step2 = tf(-20,[1,0], 'InputDelay', 2);
step3 = tf(10, [1,0], 'InputDelay', 3);

% doublet assembly process 
doublet = minreal(step1 + step2 + step3); 

% input check plot 
if args == true
    [Y, T] = impulse(doublet);

    figure 
    plot(T, Y);
    grid on 
    grid minor 
end

%% transfer function computation
% setting up transfer function law
WQinv = @(gain, control1, control2) gain * tf([1/control1, 1], [1/control2, 1]);

% setting up study vector 
gainVec     = 0.5;  % this parameter is not set as a vector but the function allows to study a vector -- see nested for loops  
control1Vec = linspace(1e+2,1e+3,10); 
control2Vec = linspace(1e+1,1e+2,20); 
%control1Vec = 900;
%control2Vec = 170;

% setting up study interval 
%tFinal = 2;

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

% evaluating response for each tuning parameter and finding the one that satisfies the magnitude constraint
for gain = gainVec
    for control1 = control1Vec 
        for control2 = control2Vec
            % applying doublet to control weight transfer function 
            
            % retreiving data in time domain and for analysis
            %[Y, T] = step(WQinv(gain, control1, control2));
            
            % time response of the system at a doublet input
            [Y, T] = lsim(WQinv(gain, control1, control2), u, t);
            
            % cheching magnitude reduction
            if max(abs(Y)) < 5 
                fprintf('magnitude constraint satisfied')
                
                figure
                plot(T, Y);

                return 
            end
        end
    end
end

end