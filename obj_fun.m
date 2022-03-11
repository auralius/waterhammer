% Define the objective function
% 
% See Eq. 7 of the paper

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