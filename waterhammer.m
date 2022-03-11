% This function simulates the waterhammer dyhnamics. 
% Main reference:
%   Chen, T., Xu, C., Lin, Q., Loxton, R., & Teo, K. L. (2015). Water 
%   hammer mitigation via PDE-constrained optimization. Control Engineering 
%   Practice, 45, 54–63. https://doi.org/10.1016/j.conengprac.2015.08.008
%
% Descriptions:
%   Simulation runs for Tf seconds, sampled at dt.
%   Pipeline is segmented into m segment and m+1 nodes.
%   The first node is at the resevoir and the last node is at the valve.
%   tau is a vector of [1 x the horizon length]. It describes the closure 
%   of the valve. The values of tau are between 0 to 1. 0 means the valve 
%   is fully open. 1 means the valve is fully close.
%   tau is discretized along the time horizon. The first element of tau is 
%   the initial condition of the valve (t=0). The last element of tau is 
%   the valve condition at t=Tf.Linear interpolation will be applied to 
%   vector tau to generate tau(t).
%

function [l, hires_tspan, hires_p, lores_tspan, lores_p] = waterhammer(tau, dt)

% The parameters
L = 200;      % m
D = 100e-3;   % m
rho = 1000;   % kg/m3
c = 1200;     % m/s
c_sqrd = c^2; % Speed up the computation
f = 0.03;     % Frinction coefficient
P = 2e5;      % Pascals

m = 24;       % Number of the pipeline segments
M = m + 1;    % Number of the pipeline nodes
dl = L / m;   % Per-segment length

% Initial values for v
u_max = 2;
v0 = u_max*ones(1, M);  % See Sec. 4 of the referenced paper

% Initial values for p, see the paper, p. 60. Section. 4
l = linspace(0, L, m + 1);
p0 = P - 2 * rho * f / D * l;

Tf = 10;
hires_dt = 0.001;  % For plotting only, so we can have a smooth plot
hires_tspan = 0:hires_dt:Tf; 
lores_tspan = 0:dt:Tf;

% The simulation starts here
[~,xsol] = ode23(@rhs, hires_tspan, [p0 v0]);
hires_p = xsol(:,1:M);                 % Timestep: hires_dt
lores_p = xsol(1:dt/hires_dt:end,1:M); % Downsamped solutions, timestep: dt

    function dxdt = rhs(t,x)
        p_dot = zeros(M, 1);
        v_dot = zeros(M, 1);

        p = x(1:M);
        v = x(M+1:end);
        ut = interp1(0:dt:Tf, tau, t, 'linear'); % This is u(t)

        % Boundary conditions
        p(1) = P;
        v(end) = u_max - u_max * ut;                   

        p_dot(end) =rho*c_sqrd/dl * (v(end-1)-v(end));
        v_dot(1) = 1/(rho*dl)*(P-p(2)) - f/(2*D)*v(1)*abs(v(1));

        % Dynamics update                
        i = 2 : M-1; % Pipe segments
        v_dot(i) = -1/rho * (p(i+1)-p(i)) / (dl) - f/(2*D) .* ...
            abs(v(i)).*v(i);
        p_dot(i) = -rho*c_sqrd * (v(i)-v(i-1)) / (dl);
                 
        dxdt = [p_dot; v_dot]; 
    end
end
