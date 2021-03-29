function [constr, z_out] = qual_mu_goal(x, lb_x, up_x, lb_y, ub_y)

[bds_x1, z_x1] = qual_mu(x(:,1), 1, lb_x);
[bds_x2, z_x2] = qual_mu(x(:,1), -1, -up_x);
[bds_y1, z_y1] = qual_mu(x(:,2), 1, lb_y);
[bds_y2, z_y2] = qual_mu(x(:,2), -1, -ub_y);
constr = [bds_x1; bds_y1; bds_x2; bds_y2];

N = size(x, 1);
z_out = binvar(N, 1);
for t = 1:N   
    [constr_and, z_out(t)] = qual_and([z_x1(t); z_y1(t); z_x2(t); z_y2(t)]);
    constr = [constr; constr_and];
end
end

