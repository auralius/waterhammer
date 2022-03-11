% Auralius Manurung, ME, Universitas Pertamina
%
% Based on the paper by:
%
% Tehuan Chen, Chao Xu, Qun Lin, Ryan Loxton, Kok Lay Teo,
% Water hammer mitigation via PDE-constrained optimization,
% Control Engineering Practice,
% Volume 45, 2015, pp. 54-63
%
% Here we will  find the optimal policy to close the valve whili minimizing
% the water-hammer effects.
%

clear all;
clc;
close all;

%%
if max(size(gcp)) == 0 % parallel pool needed
    parpool % create the parallel pool
end

%% SELECT THE SOLVER:
%SOLVER = 'pdfo';
SOLVER = 'sqp';
%SOLVER = 'ps';

%%
% Define the horizon
dt = 1;
Tf = 10;
t = 0:dt:Tf;
N = length(t); % Horizon length

% For N horizons, we have (N-1) inputs
lb = 0:1/(N-1):1;  % ;ower bounds
ub = ones(1,N);    % upper bounds
ub(1) = 0;

% Lower bounds as ICs
tau_0 = lb; 

target = @(tau)(obj_fun_runner(tau, dt));

tic

% Run the optimization
if strcmp(SOLVER, 'sqp')
    opts = optimoptions(@fmincon, ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'UseParallel', true, ...
        'SpecifyObjectiveGradient', true);
    tau_opt = fmincon(target, tau_0, [], [], [], [], lb, ub, [], opts);;    

elseif strcmp(SOLVER, 'ps')
    opts = optimoptions('patternsearch', ...
        'Display', 'iter', ...   
        'UseCompletePoll', true, ...
        'UseCompleteSearch', true, ...
        'UseParallel', true, ...
        'SearchFcn', @GPSPositiveBasis2N);
    tau_opt = patternsearch(target, tau_0, [], [], [], [], lb, ub, [], opts);

elseif strcmp(SOLVER, 'pdfo')
    opts = struct();
    opts.quiet = false;
    tau_opt = pdfo(target, tau_0, [], [], [], [], lb, ub, [], opts);
end

toc

% Simulate the optimum control inputs and compare the results
[~, t1, p_data_optimized, ~, ~] = waterhammer(tau_opt, dt);
[~, t2, p_data_unoptimized, ~, ~] = waterhammer(0:1/(N-1):1, dt); % This is contant closure rate

% Plot the results
figure
hold on
plot(t2, p_data_unoptimized(:,end));
plot(t1, p_data_optimized(:,end));
xlabel('Time (s)')
ylabel('P (Pa)')
legend('Constant closure-rate', 'Optimal closure-rate', 'Location', 'best')
set(gca,'fontname','times', 'FontSize', 12)  % Set it to times

figure
hold on 
plot(0:dt:Tf, 0:1/(N-1):1);
plot(0:dt:Tf, tau_opt);
xlabel('Time (s)')
ylabel('Valve Closing ($\tau$)', 'Interpreter','latex');
legend('Constant closure-rate', 'Optimal closure-rate', 'Location', 'best')
set(gca,'fontname','times', 'FontSize', 12)  % Set it to Times

