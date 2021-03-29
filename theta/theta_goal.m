function [constr, theta] = theta_goal(x, x_lb, x_ub, y_lb, y_ub)

[bds_x1, theta_x1] = theta_p(x(:,1), 1, x_lb);
[bds_x2, theta_x2] = theta_p(x(:,1), -1, -x_ub);
[bds_y1, theta_y1] = theta_p(x(:,2), 1, y_lb);
[bds_y2, theta_y2] = theta_p(x(:,2), -1, -y_ub);
constr = [bds_x1; bds_x2; bds_y1; bds_y2];

N = size(x, 1);
theta = sdpvar(N, 1);
for t=1:N
    [c, theta(t)] = quant_and([theta_x1(t); theta_x2(t); theta_y1(t); theta_y2(t)]);
    constr = [constr; c];
end