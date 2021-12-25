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
                   'Algorithm','sqp');
%                   'UseParallel','always');

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

figure
hold on 
plot(0:1:10, 0:0.1:1);
plot(0:1:10, tau_opt);
xlabel('Time (s)')
ylabel('Valve Closing')
legend('Constant closure-rate', 'Optimal closure-rate', 'Location', 'best')

%% ------------------------------------------------------------------------
%  Define the objective functions
%  ------------------------------------------------------------------------
function residuals = obj_fun(tau)

[t, l, p_data] = waterhammer_lo_time_res(tau);

N = size(p_data,1); % Should be 11
M = size(p_data,2); 

gamma = 2;
T = 10;
L = 200;
dt = 0.00001;
dl = 12.5;

p_hat = ones(N,1) *2e5;
P_bar = 1e5;
p_at_terminus_over_time = p_data(:, end);

delta = ((p_at_terminus_over_time - p_hat) ./ P_bar) .^ (2*gamma);
Sum1 = sum(delta)*dt/T;

delta = ((p_data - p_hat) ./ P_bar) .^ (2*gamma);
Sum2 = sum(sum(delta))*dl*dt / (L*T);

residuals = Sum1 + Sum2;

end