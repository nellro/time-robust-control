function [constr, theta, z_p] = theta_goal_lite(x, x_lb, x_ub, y_lb, y_ub)

[c1, z_p] = qual_mu_goal(x, x_lb, x_ub, y_lb, y_ub);
[c2, theta] = theta_plus(z_p);

constr = [c1; c2];