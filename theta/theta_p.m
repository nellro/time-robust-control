function [constr, theta, z_p] = theta_p(x, a, b)

[bds, z_p] = qual_mu(x, a, b);
[c, theta] = theta_plus(z_p);

constr = [bds; c];
end