function [constr, r_and] = quant_mu_goal(x, lb_x, up_x, lb_y, ub_y)

r_x1 = quant_mu(x(:,1), 1, lb_x);
r_x2 = quant_mu(x(:,1), -1, -up_x);
r_y1 = quant_mu(x(:,2), 1, lb_y);
r_y2 = quant_mu(x(:,2), -1, -ub_y);

N = size(x, 1);
r_and = sdpvar(N, 1);
constr = [];
for t = 1:N   
    [constr_and, r_and(t)] = quant_and([r_x1(t); r_y1(t); r_x2(t); r_y2(t)]);
    constr = [constr; constr_and];
end
end

