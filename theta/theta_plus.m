function [constr, theta] = theta_plus(z)
N = size(z, 1);
constr = [];

%% tau
theta = sdpvar(N, 1);
counter_ones = sdpvar(N+1, 1);
counter_zeros = sdpvar(N+1, 1);

counter_ones_shifted = sdpvar(N, 1);
counter_zeros_shifted = sdpvar(N, 1);

counter_ones(N+1) = 0; 
counter_zeros(N+1) = 0;
for t=N:-1:1
   [c1, counter_ones(t)]  = bool_int_product(z(t),   counter_ones(t+1) +1,  0, N);
   [c2, counter_zeros(t)] = bool_int_product(1-z(t), counter_zeros(t+1)-1, -N, 0);
   constr = [constr; c1; c2];
end
for t=1:N
    counter_ones_shifted(t) = counter_ones(t) - z(t);
    counter_zeros_shifted(t) = counter_zeros(t) + (1-z(t));
    theta(t) = counter_ones_shifted(t) + counter_zeros_shifted(t);
end
end