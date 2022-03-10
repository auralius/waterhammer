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
% second. The pipline is segemnted into m equally spaced seggements,
% resulting in m+1 measurement nodes. 
%
% The waterhammer function takes input in a vector (1xN), where N is the 
% horizoon length. Each element describes the condition of valve, with 0 
% means the valve is fully open and 1 means the valve is fully closed.
%
% 
% This file also shows the measurement points along the pipeline.
%

clc
close all
clear

%% Define the horizon
dt = 1;
Tf = 10;
t = 0:dt:Tf;
N = length(t); % Horizon length

%%
tau = 0:1/(N-1):1; % contant closure rate
[l, t_hi, P_hi, t_lo, P_lo] = waterhammer(tau, dt);


M = size(P_hi,2);

for k = 1:M
    s = ['Pipeline node #' num2str(k-1)];
    figure;
    plot(t_hi,P_hi(:,k))
    hold on
    plot(t_lo,P_lo(:,k), '--rs', 'MarkerEdgeColor', 'r', 'MarkerSize',10);

    title(s);
    ylabel('Pressure (Pa)')
    xlabel('Time(s)')
    legend('','Measurement points', 'Location','best')
end
set(gca,'fontname','times', 'FontSize', 12)  % Set it to times

figure
plot(l, P_hi(end,:), '--rs', 'MarkerEdgeColor', 'r', 'MarkerSize',10);
xlabel('$\ell$', 'Interpreter','latex');
ylabel('$p(\ell)$', 'Interpreter','latex');
title('Pressure along the pipe after 10 seconds');
set(gca,'fontname','times', 'FontSize', 12)  % Set it to times

