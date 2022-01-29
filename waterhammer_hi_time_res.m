% This function return p_data, which is a 1000001xM matrix. 
% Simulation runs for 10 seconds, sampled at 0.00001s.
% All data are logged!
%
% M is number of the nodes located along the pipeline where presure 
% measurements are being perforomed. 
%
% The first node is at the resevoir and the last node is at the valve.
%
% As for the rows, the first row is the pressure taken at t = 0;
% The second row is the pressure taken at t = 0 + dt;
% The third row is the pressure taken at t = 0 + 2dt;
% The last row is the pressure taken at t = 10;
%
% tau is a vector of 1x11. Its value is between 0 to 1. 
% 0 means the valve is fully open.
% 1 means the valve is fully close.
%
% The first element of tau describe the valve condition at t = 0.
% The second element of tau describe the valve condition at t = 1.
% Lastly, the rleventh element of tau describe the valve condition at 
% t = 10.
%
% Linear interpolation will be applied to vector tau
%
function [t, l, p_data] = waterhammer_hi_time_res(tau)

% The parameters
L = 200;    % m
D = 100e-3; % m
rho = 1000; % kg/m3
c = 1200;   % m/s
f = 0.03;
P = 2e5;    % Pa

m = 16;       % Even integer, number of the pipeline segments
M = m + 1;    % Number of the pipeline nodes
dt = 0.00001; % Crazy small number ;-)
dl = L / m;

% Initial values for v
u_max = 2;
v = u_max*ones(1, m+1);  % See Sec. 4 of the referenced paper

% Initial values for p
l = linspace(0, L, m + 1);
p = P - 2 * rho * f / D * l;

T = 10;
t = 0:dt:T;
N = length(t);

TAU = interp1(0:10, tau, t, 'linear');

p_data = zeros(N, M);

p_dot = zeros(1, m+1);
v_dot = zeros(1, m+1);

% The simulation starts here
idx = 1;

for j = 1 : N    
    i = 2 : M-1;
    v_dot(i) = -1/rho * (p(i+1)-p(i-1)) / (2*dl) - f/(2*D) .* ...
        abs(v(i)).*v(i);
    p_dot(i) = -rho*c^2 * (v(i+1)-v(i-1)) / (2*dl);

    p = p_dot*dt + p;
    v = v_dot*dt + v;

    % Apply BC
    v(M) = u_max - u_max * TAU(j);  % Apply the inputs                    
    p(M) = p(M) + dt * rho*c^2/dl * ( v(M-1)-v(M) );

    v(1) = v(1) + dt * ( 1/(rho*dl)*(P-p(2)) - f/(2*D)*v(1)*abs(v(1)) );
    p(1) = P;

    p_data(idx,:) = p;
    idx = idx + 1;    
end
end
