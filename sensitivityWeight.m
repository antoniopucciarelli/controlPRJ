% tuning sensitivity weight to be used in the overall plant tuning via systune 
% aim: 
% -- compute: A, M, omega_b that allow to satisfy the 2nd order response of the system 
%

% constraint values 
damp = 0.9;
omega_n = 10; 

% setting up tolerances for high frequency study 
tol = -79;
% setting up evaluation frequency for the steady state analysis
freq = 1e-5;

% setting up possible intervals for the parameters 
dimA = 10;
dimM = 10; 
dimOmega_b = 10; 

% from theory 
Mideal = 1/damp;

% setting up study vectors:
Avec       = linspace(5e-5, 1e-4, dimA);             
Mvec       = linspace(Mideal, Mideal*1.1, dimM);
omega_bVec = linspace(0.5*omega_n, 0.6*omega_n, dimOmega_b);

% transfer function declaration 
WPinv = @(A, M, omega_b) tf([1, omega_b*A], [1/M, omega_b]);

% tuning loop 
test = 0;
for A = Avec
    for M = Mvec 
        for omega_b = omega_bVec
            test = test + 1;
            [Gm,Pm,Wcg,Wcp] = margin(WPinv(A, M, omega_b));
            fprintf('test(%d)\ninput:\n\tA = %f \n\tM = %f \n\tomega = %f\noutput:\n\tGm = %f\n\tPm = %f\n\tWcg = %f\n\tWcp = %f\n\n', ...
                test, A, M, omega_b, Gm, Pm, Wcg, Wcp);
            
            [magnitude, phase] = bode(WPinv(A, M, omega_b), freq); 
            magnitude = 20 * log10(magnitude);
            
            if Wcp > 10 && magnitude <= tol 
                fprintf('Result\ntarget:');
                fprintf('\n\tcrossover frequency -> Wcp > 10\n\t\tWcp = %f rad/s\n', Wcp);

                fprintf('\n\tsteady state error close to 0 (so db -> -Inf)\n');
                fprintf('\t\tmagnitude (db) @ omega = %f rad/s\n\t\tmagnitude = %f db\n\n', freq, magnitude);
                break 
            end
        end
    end
end

figure(1);
bodemag(WPinv(A, M, omega_b));
grid on
grid minor 

fprintf('function parameters after tuning:\n\t* A = %f\n\t* M = %f\n\t* omega_b = %f\n', A, M, omega_b);
