function [constr, y] = bool_int_product(b, x, x_lb, x_ub) % b is boolean, x is integer
N = size(b, 1);
y = sdpvar(N, 1);

constr = [];
for t = 1:N
    c1 = x_lb * b(t) <= y(t)<= x_ub * b(t); 
    c2 = x(t) - x_ub*(1-b(t)) <= y(t) <= x(t) - x_lb*(1-b(t));
    constr = [constr; c1; c2];
end
end