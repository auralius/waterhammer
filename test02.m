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

% Run the optimization
if strcmp(SOLVER, 'sqp')
    opts = optimoptions(@fmincon, ...
        'Display', 'none', ...
        'Algorithm', 'sqp', ...
        'SpecifyObjectiveGradient', true);
    ms = MultiStart('Display','iter', 'UseParallel', true);
    problem = createOptimProblem('fmincon', 'x0', tau_0, ...
              'objective', target, 'lb', lb, 'ub', ub, 'options', opts);
    tau_opt = run(ms, problem, 30);    

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

% Simulate the optimum control inputs and compa

[~, t1, p_data_optimized, ~, ~] = waterhammer(tau_opt, dt);
[~, t2, p_data_unoptimized, ~, ~] = waterhammer(0:1/(N-1):1, dt); % This is contant closure rate

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

%% ------------------------------------------------------------------------
%  Define the objective functions
%  ------------------------------------------------------------------------
function [J, grad_J] = obj_fun_runner(tau, dt)

[~, ~, ~, ~, p] = waterhammer(tau, dt);
J = obj_fun(p, dt);

if nargout > 1 % gradient required
    epsilon = 1e-3;
    epsilon_inv = 1/epsilon;

    grad_J = zeros(length(tau), 1);

    for k = 1 : length(tau)
        tau_ = tau;
        tau_(k) = tau(k) + epsilon * 1i;
        [~, ~, ~, ~, p_] = waterhammer(tau_, dt);
        J_ = obj_fun(p_, dt);
        grad_J(k) = imag(J_).*epsilon_inv;
    end
end

end

%% See Eq. 7 of the paper
function J = obj_fun(p, dt)

gamma   = 2;
T       = 10;
L       = 200;
m       = 16;
dl      = L/m;
p_ref   = 2e5;
p_toll  = 1e5;

% Terminal
delta1 = ((p(:,end) - p_ref) ./ p_toll) .^ (2*gamma);
Sum1 = 1/T*sum(delta1)*dt;

% Stage
delta2 = ((p - p_ref) ./ p_toll).^ (2*gamma);
Sum2 = 1/(L*T)*sum(sum(delta2))*dl*dt;

J = Sum1 + Sum2;
end