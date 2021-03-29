% PHI1

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
constraints = [constraints; c1];

a1 = 0;
b1 = 20;
[constr, theta_g] = quant_globally(theta1, 1, a1, b1);
constraints = [constraints; constr];


theta_phi = theta_g;
objective = theta_phi;
%objective = L-sum(abs(u));

options = sdpsettings('debug', 1,...
'solver', 'gurobi', ...
    'verbose',1);
sol = optimize(constraints, -objective, options);

R = value(theta_phi);
fprintf('Right time rob: %i(time units)\n', R);
