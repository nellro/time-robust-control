function [constr, r_out] = quant_and(r)
M = 1e6;

m = size(r, 1);
r_out = sdpvar(1, 1);
b = binvar(m, 1);

constr = [];
for i = 1:m
    constr = [constr; r(i) - M*(1-b(i)) <= r_out <= r(i)];
end
constr = [constr; sum(b) == 1];

% constr = r - M*(1-b) <= r_out <= r;
% constr = [constr; sum(b) == 1];