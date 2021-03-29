function [constr, z] = qual_mu(x, a, b)
% ax_t >= b 
% therefore, ax - b >= 0
M = 1e6;
eps = 1e-4;

N = size(x, 1);
z = binvar(N, 1);
constr=[];
for k = 1:N
    constr = [constr; M*(z(k)-1) + eps <= a*x(k) - b <= (M+eps)*z(k) - eps]; 
end

% constr = -M*(1-z) + eps <= a*x - b <= M*z- eps;
