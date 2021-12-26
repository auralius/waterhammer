% Auralius Manurung, ME, Universitas Pertamina
%
% Based on the paper by:
%
% Tehuan Chen, Chao Xu, Qun Lin, Ryan Loxton, Kok Lay Teo,
% Water hammer mitigation via PDE-constrained optimization,
% Control Engineering Practice,
% Volume 45, 2015, pp. 54-63
%
% Here we will call to a matlab function that simulates the pipeline for 10
% second. The pipline is segemnted into 16 equally spaced seggements,
% resulting in 17 measurement nodes. 
%

clear all;
clc;
close all;

N = 11;

% Initial input guess, some arbitrary values
% For N horizons, we have (N-1) inputs
tau_0 = 0.5*ones(1,N); 
lb = zeros(1,N);
ub = ones(1,N);

lb(1) = 0;
ub(1) = 0;

lb(end) = 1;
ub(end) = 1;

tau_0(1) = 0;
tau_0(end) = 1;

% Run the optimization
opts = optimoptions(@fmincon, ...
                   'Display','Iter',...
                   'Algorithm','sqp', ...
                   'UseParallel','always');

tic
tau_opt = fmincon(@obj_fun, tau_0, [], [], [], [], lb, ub, [], opts);
toc

% Simulate the optimum control inputs and compa

[t1, l, p_data_optimized] = waterhammer_hi_time_res(tau_opt);
[t2, l, p_data_unoptimized] = waterhammer_hi_time_res(0:0.1:1);

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
plot(0:1:10, 0:0.1:1);
plot(0:1:10, tau_opt);
xlabel('Time (s)')
ylabel('Valve Closing ($\tau$)', 'Interpreter','latex');
legend('Constant closure-rate', 'Optimal closure-rate', 'Location', 'best')
set(gca,'fontname','times', 'FontSize', 12)  % Set it to times

%% ------------------------------------------------------------------------
%  Define the objective functions
%  ------------------------------------------------------------------------
function residual = obj_fun(tau)

[~, ~, p_data] = waterhammer_lo_time_res(tau);

gamma   = 2;
T       = 10;
L       = 200;
dt      = 0.00001;
dl      = 12.5;
p_ref   = 2e5;
p_toll   = 1e4;

% Intial 
delta0 = ((p_data(:, 1) - p_ref) ./ p_toll) .^ (2*gamma);
Sum0 = 1/T*sum(delta0)*dt;

% Terminal
delta1 = ((p_data(:, end) - p_ref) ./ p_toll) .^ (2*gamma);
Sum1 = 1/T*sum(delta1)*dt;

% Stage
delta2 = ((p_data - p_ref) ./ p_toll).^ (2*gamma);
Sum2 = 1/(L*T)*sum(sum(delta2))*dl*dt;

residual = Sum0 + Sum1 + Sum2;
end