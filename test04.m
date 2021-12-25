clc
clear all
close all

load run1.mat;


% Simulate the optimum control inputs
[t1, p_data_optimized] = waterhammer_hi_time_res(tau_opt);
[t2, p_data_unoptimized] = waterhammer_hi_time_res(0:0.1:1);
