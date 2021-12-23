% Auralius Manurung, ME, Universitas Pertamina
%
% Based on the paper by:
%
% Tehuan Chen, Chao Xu, Qun Lin, Ryan Loxton, Kok Lay Teo,
% Water hammer mitigation via PDE-constrained optimization,
% Control Engineering Practice,
% Volume 45, 2015, pp. 54-63

clc
close all
clear

% The parameters
L = 200;    % m
D = 100e-3; % m
rho = 1000; % kg/m3
c = 1200;   % m/s
f = 0.03;
P = 2e5;    % Pa

m = 16;       % Even integer, number of the pipeline segments
N = m + 1;    % Number of the pipeline nodes
dt = 0.00001; % Crazy small number ;-)
dl = L / m;

% Initial values for v
u_max = 2;
v = u_max*ones(1, m+1);  % See Sec. 4 of the referenced paper

% Initial values for p
l = linspace(0, L, m + 1);
p = P - 2 * rho * f / D * l;

hfig = figure;

subplot(2,1,1)
h1 = plot(0,0);
xlabel('$\ell$', 'Interpreter', 'latex')
ylabel('$v(\ell)$', 'Interpreter', 'latex')
htext = text(L/2, u_max/2, '');
ylim([0 u_max])

subplot(2,1,2);
h2 = plot(0,0);
xlabel('$\ell$', 'Interpreter', 'latex')
ylabel('$p(\ell)$', 'Interpreter', 'latex')
ylim([0 P+0.2*P])

T = 10;
t = 0:dt:T;
p_terminus = zeros(1, length(t));

p_dot = zeros(1, m+1);
v_dot = zeros(1, m+1);

% The simulation starts here
for k = 1 : length(t)                         
    
    i = 2 : N-1;
    v_dot(i) = -1/rho * (p(i+1)-p(i-1)) / (2*dl) - f/(2*D) .* ...
               abs(v(i)).*v(i);
    p_dot(i) = -rho*c^2 * (v(i+1)-v(i-1)) / (2*dl);  

    p = p_dot*dt + p;
    v = v_dot*dt + v;

    % Apply BC
    v(N) = u_max - u_max / T * (k-1) * dt;           % See Sec. 4.1
    p(N) = p(N) + dt * rho*c^2/dl * ( v(N-1)-v(N) );        

    v(1) = v(1) + dt * ( 1/rho*dl*(P-p(2)) - f/(2*D)*v(1)*abs(v(1)) );    
    p(1) = P;                                

    if (rem((k-1), 0.1/dt) == 0) % Write to a GIF file every 0.1s
        set(h1, 'XData', l, 'YData',v);
        set(h2, 'XData', l, 'YData',p);
        drawnow;       
        htext.String = ['t = ', num2str((k-1)*dt)];
        write2gif(hfig, k, 'waterhammer.gif', 0.1);
    end

    % Log pressure at the valve
    p_terminus(k) = p(m);
end

figure
plot(t, p_terminus);
title('Pressure at pipeline terminus')
xlabel('Time (s)')
ylabel('Pressure (Pa)')