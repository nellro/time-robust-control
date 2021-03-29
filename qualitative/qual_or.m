function [constr, z_out] = qual_or(z)
z_out = binvar(1, 1);

m = size(z, 1);
constr = [];
for ii = 1:m
    constr = [constr; z_out >= z(ii)];
end
constr = [constr; z_out <= sum(z)];