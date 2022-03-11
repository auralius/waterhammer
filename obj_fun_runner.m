% Preliminary process before calling the objective function.
% Before we can calculate the cost, we must calculate the system states for
% the given decission variables. The cost function is a function of the 
% state variables.

function [J, grad_J] = obj_fun_runner(tau, dt)

[~, ~, ~, ~, p] = waterhammer(tau, dt);
J = obj_fun(p, dt);

if nargout > 1 % gradient required
    % Using the Complex-Step Derivative Approximation method
    epsilon = 1e-3; % a small number to perform perturbation
    epsilon_inv = 1/epsilon;

    grad_J = zeros(length(tau), 1);

    for k = 1 : length(tau)
        tau_ = tau;
        tau_(k) = tau(k) + epsilon * 1i;
        [~, ~, ~, ~, p_] = waterhammer(tau_, dt); % Do the perturbation!
        J_ = obj_fun(p_, dt);                     % Perturbed results
        grad_J(k) = imag(J_).*epsilon_inv;
    end
end

end