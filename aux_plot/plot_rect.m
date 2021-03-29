function plot_rect(a, b, v1, v2, color, alph)
h = fill_between([a,b], v1*ones(1,2), v2*ones(1,2));
h.FaceColor = color;
alpha(h, alph);
end