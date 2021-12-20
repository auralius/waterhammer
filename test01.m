% Tehuan Chen, Chao Xu, Qun Lin, Ryan Loxton, Kok Lay Teo,
% Water hammer mitigation via PDE-constrained optimization,
% Control Engineering Practice,
% Volume 45,
% 2015,
% Pages 54-63

clc
close all
clear

% Parameters
L = 200;    % m
D = 100e-3; % m
rho = 1000; % kg/m3
c = 1200;   % m/s
f = 0.03;
P = 2e5;    % Pa

Nl = 11;
dt = 0.0001;
dl = L/(Nl-1);

u_max = 2;
v = u_max*ones(1, Nl);
l = linspace(0, L, Nl);
p = P-2*rho*f/D*l;

hfig = figure;

subplot(2,1,1)
h1 = plot(0,0);
xlabel('$\ell$', 'Interpreter', 'latex')
ylabel('$v(\ell)$', 'Interpreter', 'latex')
htext = text(L/2,0, '');
ylim([0 u_max+1])

subplot(2,1,2);
h2 = plot(0,0);
xlabel('$\ell$', 'Interpreter', 'latex')
ylabel('$p(\ell)$', 'Interpreter', 'latex')
ylim([0 500*P])

T = 10;
t = 0:dt:T;
p_terminus = zeros(1, length(t));

% The simulation starts here
for j = 1 : length(t)      % time-wise iteration
    p_next = p;
    v_next = v;

    k = 2 : Nl-1;
    v_next(k) = -1/rho * (p(k+1)-p(k-1)) / (2*dl) - f/(2*D) .* abs(v(k)).*v(k);
    p_next(k) = -rho*c^2 * (v(k+1)-v(k-1)) / (2*dl);  

    p = p_next*dt + p;
    v = v_next*dt + v;

    % Apply BC
    v(1) = u_max;                           % Dirchlet
    v(end) = u_max - u_max/T*(j-1)*dt;      % Dirchlet
    p(1) = P;                               % Dirchlet
    p(end) = p(end-2);                      % Neumann

    if (rem((j-1), 0.1/dt)==0)
        set(h1, 'XData', l, 'YData',v);
        set(h2, 'XData', l, 'YData',p);
        drawnow;
        htext.Position = [L/2 (max(v)+min(v))/2 0];
        htext.String = ['t = ', num2str((j-1)*dt)];
        write2gif(hfig, j, 'waterhammer.gif', 0.1);
    end

    p_terminus(j) = p(end);
end

figure
plot(t, p_terminus);
title('Pressure at pipeline terminus')
xlabel('Time (s)')
ylabel('Pressure (Pa)')