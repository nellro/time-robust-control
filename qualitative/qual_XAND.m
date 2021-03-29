function [constr, z_out] = qual_XAND(z1, z2)

% [constr_and, z_and] = qual_and([z1; z2]);
% [constr_and_neg, z_and_neg] = qual_and([qual_neg(z1); qual_neg(z2)]);
% 
% [constr, z_or] = qual_or([z_and; z_and_neg]);
% 
% constr = [constr_and; constr_and_neg; constr];
% z_xand = z_or;

z_out = binvar(1, 1);
c1 = z_out >= 1 - z1 - z2;
c2 = z_out >= -(1 - z1 - z2);
c3 = z_out <= 1 + z1 - z2;
c4 = z_out <= 1 - z1 + z2;
constr = [c1; c2; c3; c4];


