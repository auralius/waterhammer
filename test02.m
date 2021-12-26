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
% The waterhammer function takes input in a vector (1x11). Ecah element
% describes the condition of valve, with 0 means the valve is fully open
% and 1 means the valve is fully closed.
%
% This is for testing waterhammer_lo_time_res and waterhammer_hi_time_res
% functions
% 
% This also shows the measurement points along the pipeline
%

clc
close all
clear

%%
[t_lo, l, P_lo] = waterhammer_lo_time_res(0:0.1:1);
[t_hi, l, P_hi] = waterhammer_hi_time_res(0:0.1:1);

M = size(P_lo,2);

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
title('Pressure at t = 10s');
set(gca,'fontname','times', 'FontSize', 12)  % Set it to times

