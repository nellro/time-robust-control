function plot_predicate(p, color)
eps = 0.0001;

for t = 0:size(p, 1)-2
    p_current = value(p(t+1));
    p_next = value(p(t+2));
    
    if abs(p_current) <= eps && abs(p_next-1) <= eps || ...
            abs(p_current-1) <= eps && abs(p_next) <= eps
        plot([t,t+1], [p_current, p_next], ':', 'color', color, 'linewidth', 2);
    else
        plot([t,t+1], [p_current,p_next], 'color', color, 'linewidth', 2);
    end
end
end