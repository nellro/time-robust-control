function [constr, r_out] = quant_or(r)
M = 1e6;

m = size(r, 1);
r_out = sdpvar(1, 1);
b = binvar(m, 1);

constr = [];
for i = 1:m
    constr = [constr; r(i) <= r_out <= r(i) + M*(1-b(i))];
end
constr = [constr; sum(b) == 1];

% constr = r <= r_out <= r + M*(1-b) ;
% constr = [constr; sum(b) == 1];