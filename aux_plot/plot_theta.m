function plot_theta(theta, color)
eps = 0.0001;

for t = 0:size(theta, 1)-2
    theta_current = value(theta(t+1));
    theta_next = value(theta(t+2));
    
    if abs(theta_current) <= eps
        plot([t,t+1], [theta_current, theta_next], ':', 'color', color, 'linewidth', 2);
    else
        plot([t,t+1], [theta_current, theta_next], 'color', color, 'linewidth', 2);
    end
end
end