function plot_chi(p, color)
eps = 0.0001;

N = size(p, 1);
for t = 0:N-2
    p_current = value(p(t+1));
    p_next = value(p(t+2));
    
    chi_current = 2*p_current - 1;
    chi_next = 2*p_next - 1; 
    
    if abs(p_current) <= eps && abs(p_next-1) <= eps || ...
            abs(p_current-1) <= eps && abs(p_next) <= eps
        plot([t,t+1], [chi_current, chi_next], ':', 'color', color, 'linewidth', 2);
    else
        plot([t,t+1], [chi_current,chi_next], 'color', color, 'linewidth', 2);
    end
end
end