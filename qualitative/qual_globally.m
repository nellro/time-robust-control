function [constr, z_out] = qual_globally(z, t, a, b)
atN = t+a;
btN = t+b;

if btN > size(z, 1)
    error('Oops, error with calling Finally. Value t+I exceeds signal length');
end

[constr, z_out] = qual_and(z(atN:btN));
end