function [A, M, omega_b] = sensitivityWeight(damp, omega_n, args) 
%   AEROSPACE CONTROL SYSTEM PROJECT -- AY 2021/2022
%   authors:
%       * Mate-Erik Moni 
%       * Antonio Pucciarelli 
%       * Atefeh Esmaelizadeh Rostam
%   
%   problem description: 
%   --- analysis, design and verification of a quadrirotor, ANT-R, single axis attitude control system 
%
%   this function: 
%   --- tuning sensitivity weight to be used in the overall plant tuning via systune  
%   --- compute: A, M, omega_b that allow to satisfy the 2nd order response of the system 
%   --- this is an alternative sensitivity weight function with respect to the one given following 'directly' the assignment
%       this because:
%       --- most of the times it is better using a simpler transfer function with the same important behaviours of the more complicated ones
%       --- most of the times systune gives problem with the analysis of complicated transfer functions
%   --- transfer function law: x0 * (s + x1) / (s + x2) -> 1 zero + 1 pole transfer function 
%       this because:  
%       --- need to catch the behaviour of a 2nd order transfer function (already declared)
%
%   input:
%   --- damp: 2nd order function damping ratio 
%   --- omega_n: 2nd order function natural frequency 
%   --- args: boolean value for the print of results
%   
%   output:
%   --- A: low frequency value of the sensitivity 
%   --- M: M >= 1/damping ratio (damp)
%   --- omega_b: lowerbound bandwidth sensitivity
%

% setting up tolerances for high frequency study 
tol = 20 * log10(1e-4);
% setting up evaluation frequency for the steady state analysis -- this frequency is extremely low 
freq = 1e-5;

% setting up possible intervals for the parameters 
dimA       = 10;
dimM       = 10; 
dimOmega_b = 10; 

% from theory: Multivariable feedback control [table. 2.1] 
% --- table 2.1 relates Ms to 2nd order sensitivity transfer functions 
% --- table 2.1 suggests values around 1.15 and 1.22
% --- since problems in systune() -> decided to use a different transfer function
% --- because there is a loop Mideal starts from 1/damp value and then it goes up 
% --- Ms = |S|_Inf 
Ms_ideal = 1.185;

% setting up study vectors:
Avec       = linspace(5e-5, 1e-4, dimA);                        % from theory: A == behaviour at low frequencies (maximum steady state tracking error)            
Mvec       = linspace(Ms_ideal*0.98, Ms_ideal*1.02, dimM);      % from theory: M > 1 behaviour at high frequencies
omega_bVec = linspace(omega_n, 1.1*omega_n, dimOmega_b);        % from theory: omega_b == bandwidth requirement (frequency interval where we can see important changes from the transfer function)

% transfer function declaration -- sensitivity weight
% from theory: Multivariable feedback control [eqn. 2.73]
% performance transfer function 
WPinv = @(A, M, omega_b) tf([1, omega_b*A], [1/M, omega_b]);

% tuning loop 
test = 0; % # of test counter 
for A = Avec
    for M = Mvec 
        for omega_b = omega_bVec
            % test counter update
            test = test + 1;
            
            % main quantities computation
            [Gm,Pm,Wcg,Wcp] = margin(WPinv(A, M, omega_b));

            if args == true
                % printing results
                fprintf('test(%d)\ninput:\n\tA = %f \n\tM = %f \n\tomega = %f\noutput:\n\tGm = %f\n\tPm = %f\n\tWcg = %f\n\tWcp = %f\n\n', test, A, M, omega_b, Gm, Pm, Wcg, Wcp);
            end

            % computing transfer function magnitude at freq 
            [magnitude, ~] = bode(WPinv(A, M, omega_b), freq); 
            % converting magnitude in decibel unit
            magnitude = 20 * log10(magnitude);
            
            % checking if results satisfy given constraints
            if Wcp > 10 && magnitude <= tol 
                % printing results
                fprintf('Result\ntarget:');
                fprintf('\n\tcrossover frequency -> Wcp > 10\n\t\tWcp = %f rad/s\n', Wcp);

                fprintf('\n\tsteady state error close to 0 (so db -> -Inf)\n');
                fprintf('\t\tmagnitude (db) @ omega = %f rad/s\n\t\tmagnitude = %f db\n\n', freq, magnitude);
                
                fprintf('function parameters after tuning:\n\t* A = %f\n\t* M = %f\n\t* omega_b = %f\n', A, M, omega_b);
                
                if args == true
                    % bode plot of the transfer function -- sensitivity weight function 
                    figure
                    bode(WPinv(A, M, omega_b));
                    grid on
                    grid minor 
                end 

                return 
            end
        end
    end
end

end