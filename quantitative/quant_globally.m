function [constr, r_out] = quant_globally(r, t, a, b)
M = 1e6;
atN = t+a;
btN = t+b;

if btN > size(r, 1)
    error('Oops, error with calling Globally. Value t+I exceeds signal length');
end

[constr, r_out] = quant_and(r(atN:btN));
end