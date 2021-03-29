function r = quant_mu(x, a, b)
% ax_t >= b (ax - b >= 0) 
% thus mu := ax-b

N = size(x, 1);
r = sdpvar(N, 1);
for k = 1:N
    r(k) = a*x(k) - b;
end
