% PHI5

clear all
close all

params.Nsteps = 50;
params.N = params.Nsteps;

constraints = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control variable
x = sdpvar(params.Nsteps, 1); %inputs, a variable

% Predicate Ax>=b;
[c1, theta1] = theta_p(x(:,1), 1, 0.1);
[c2, theta2] = theta_p(x(:,1), -1, -0.5);
constraints = [constraints; c1; c2];

a1 = 0;
b1 = 10;
theta_f = sdpvar(11, 1);
for t = 1:11
    [constr, theta_f(t)] = quant_finally(theta1, t, a1, b1);
    constraints = [constraints; constr];
end

[constr, theta_g] = quant_globally(theta_f, 1, 0, 10);
constraints = [constraints; constr];

[constr, theta_f2] = quant_finally(theta2, 1, 0, 20);
constraints = [constraints; constr];

[constr, theta] = quant_and([theta_g; theta_f2]);
constraints = [constraints; constr];

theta_phi = theta;
objective = theta_phi;
%objective = L-sum(abs(u));

options = sdpsettings('debug', 1,...
'solver', 'gurobi', ...
    'verbose',1);
sol = optimize(constraints, -objective, options);

R = value(theta_phi);
fprintf('Right time rob: %i(time units)\n', R);
