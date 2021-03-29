clear all
close all

params.Nsteps = 100;
params.N = params.Nsteps-1;

constraints = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control variable
u = sdpvar(params.N, 1); %inputs, a variable
u_max = 0.2;
u_bds = -u_max*ones(params.N, 1) <= u <= u_max*ones(params.N, 1);
constraints = [constraints; u_bds];

%% Dynamics
x0 = [0, 0];
x = x0;
Ts = 1;
A = [1 Ts; 0 1];
B = [Ts*Ts./2; Ts];
for k = 1:params.N
    x_k = x(end, :)';
    u_k = u(k);
    
    %wk = [0; 0.01]* normrnd(0,1);
    x_kp1 = A*x_k + B*u_k;
    x = [x; x_kp1'];
end

vel_max = 1.5;
vel_bds = -vel_max*ones(params.Nsteps, 1) <= x(:, 2) <= vel_max*ones(params.Nsteps, 1);
constraints = [constraints; vel_bds];
%% Globally_[a, b] z>=2
% Predicate Ax>=b;
alt_lb = 20;
[bds_x1, theta1, z_p1] = theta_p(x(:,1), 1, alt_lb);
constraints = [constraints; bds_x1];

% vel_ub = 0.01;
% [bds_x2, z2] = theta_p(x(:,2), -1, -vel_ub);
% constraints = [constraints; bds_x2];
% 
% vel_lb = -0.01;
% [bds_x3, z3] = qual_mu(x(:,2), 1, vel_lb);
% constraints = [constraints; bds_x3];
% 
% z4 = binvar(params.Nsteps, 1);
% for t=1:params.Nsteps
%     [constr, z4(t)] = qual_and([theta1(t); z2(t); z3(t)]);
%     constraints = [constraints; constr];
% end

% Globally 1
a1 = 20;
b1 = 30;
n_g1 = params.Nsteps-b1;
[constr, theta_g1] = quant_globally(theta1, 1, a1, b1);
constraints = [constraints; constr];
%------------------------------

alt_ub = 10.;
[bds_x5, theta5, z_p5] = theta_p(x(:,1), -1, -alt_ub);
constraints = [constraints; bds_x5];

a2 = 60;
b2 = 70;
n_g2 = params.Nsteps-b2;
[constr, theta_g2] = quant_globally(theta5, 1, a2, b2);
constraints = [constraints; constr];

%--------------------------
n_phi = min(n_g1, n_g2);
[constr, theta_phi] = quant_and([theta_g1; theta_g2]);
constraints = [constraints; constr];

%-------------------------------------------------
objective = theta_phi;
%objective = L-sum(abs(u));

options = sdpsettings('solver','gurobi');
sol = optimize(constraints, -objective, options);

%%
R = value(theta_phi);
fprintf('Right time rob: %i(time units)\n', R);
%
color_red = [200./255, 0, 0];
color_blue = [51./255, 102./255, 1];
color_green = [.0 .875 .0];
color_orange = [1 102./255 0];
%%%%%%

figure
ymax = 40;
ymin = -60;

plt_setup = {'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'right'};
hold on
subplot(3, 1, 1);
hold on
plot(0:(params.Nsteps-1), value(x(:, 1)), 'color', color_blue, 'linewidth', 2);
xline(a1, 'k--', 'a1', plt_setup{:});
xline(b1, 'k--', 'b1', plt_setup{:});
xline(a2, 'k--', 'a2', plt_setup{:});
xline(b2, 'k--', 'b2', plt_setup{:});
yline(alt_lb, ':', 'color', 'k', 'linewidth', 0.5);
yline(alt_ub, ':', 'color', 'k', 'linewidth', 0.5);

hf1 = fill_between([a1,b1],alt_lb*ones(1,2),ymax*ones(1,2));
hf1.FaceColor = color_green;
alpha(hf1, .2);
hf2 = fill_between([a2,b2],ymin*ones(1,2),alt_ub*ones(1,2));
hf2.FaceColor = color_green;
alpha(hf2, .2);
rf1 = fill_between([b1,b1+R],alt_lb*ones(1,2),ymax*ones(1,2));
rf1.FaceColor = 'y';
alpha(rf1, .5);
rf2 = fill_between([b2,b2+R],ymin*ones(1,2),alt_ub*ones(1,2));
rf2.FaceColor = 'y';
alpha(rf2, .5);
xx = (b1+1):(b1+R);
scatter(xx, value(x(xx, 1)), 10, color_green, 'filled');
xx = (b2+1):(b2+R);
scatter(xx, value(x(xx, 1)), 10, color_green, 'filled');

set(gca, 'box', 'on')
title('Position');
ylim([ymin, ymax]);

subplot(3, 1, 2);
plot(0:(params.Nsteps-1), value(x(:, 2)), 'color', color_blue, 'linewidth', 2);
yline(vel_max, '--', 'v_{max}', 'color', color_red, 'linewidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(-vel_max, '--', 'v_{min}', 'color', color_red, 'linewidth', 1.5);
%ylim([-1.1*vel_max, 1.1*vel_max]);
title('Velocity');

subplot(3, 1, 3);
plot(0:(params.N-1), value(u), 'color', color_blue, 'linewidth', 2);
title('Control Input');
ylim([-1.4*u_max, 1.4*u_max]);
yline(u_max, '--', 'u_{max}', 'color', color_red, 'linewidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(-u_max, '--', 'u_{min}', 'color', color_red, 'linewidth', 1.5);
xlabel('Time (steps)');

% 
% figure;
% n_col = 1;
% n_row = 2;
% k = 1;
% hrz = params.Nsteps;
% 
% subplot(n_row, n_col, k);
% yyaxis right
% pl1 = plot(0:size(z_p1, 1)-1, value(z_p1), 'color', color_blue, 'linewidth', 2);
% ylim([-1.1, 1.1]);
% set(gca,'YColor',color_blue);
% hold on
% yyaxis left
% pl2 = plot(0:size(theta1, 1)-1, value(theta1), 'color', color_red, 'linewidth', 2);
% ylim([-45, 45]);
% set(gca,'YColor', color_red);
% legend([pl1, pl2], {'p1', '\theta_{p1}(x)'}, 'AutoUpdate', 'off');
% xlim([0, hrz]);
% xline(a1, 'k--', 'a1', plt_setup{:});
% xline(b1, 'k--', 'b1', plt_setup{:});
% %legend('boxoff')
% k = k+1;
% %--------------
% subplot(n_row, n_col, k);
% yyaxis right
% pl1 = plot(0:size(z_p5, 1)-1, value(z_p5), 'color', color_blue, 'linewidth', 2);
% ylim([-1.1, 1.1]);
% set(gca,'YColor',color_blue);
% hold on
% yyaxis left
% pl2 = plot(0:size(theta5, 1)-1, value(theta5), 'color', color_red, 'linewidth', 2);
% ylim([-45, 45]);
% set(gca,'YColor', color_red);
% legend([pl1, pl2], {'p2', '\theta_{p2}(x)'}, 'AutoUpdate', 'off', 'Location','southeast');
% xlim([0, hrz]);
% xline(a2, 'k--', 'a2', plt_setup{:});
% xline(b2, 'k--', 'b2', plt_setup{:});


%%%%
figure;
fz = 12;
n_col = 1;
n_row = 4;
k = 1;
hrz = params.Nsteps;

subplot(n_row, n_col, k);
hold on
plot_chi(z_p1, color_red);
set(gca, 'box', 'on');
title('$$\chi_{p}(\mathbf{x}),\ \  p = z \geq 20$$', 'interpreter','latex', 'FontSize', fz);

xlim([0, hrz]);
%xlabel('Time (steps)');
xline(a1, 'k--', 'a1', plt_setup{:});
xline(b1, 'k--', 'b1', plt_setup{:});
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_theta(theta1, color_red);
set(gca, 'box', 'on');
title('$$\theta^+_{p}(\mathbf{x}),\ \  p = z \geq 20$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
xline(a1, 'k--', 'a1', plt_setup{:});
xline(b1, 'k--', 'b1', plt_setup{:});
yline(0, 'k-');
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_chi(z_p5, color_red);
set(gca, 'box', 'on');
%title(sprintf('G_{[%d,%d]} z_1', a1, b1));
title('$$\chi_{q}(\mathbf{x}),\ \  q = z \leq 10$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
xline(a2, 'k--', 'a2', plt_setup{:});
xline(b2, 'k--', 'b2', plt_setup{:});
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_theta(theta5, color_red);
set(gca, 'box', 'on');
title('$$\theta^+_{q}(\mathbf{x}),\ \  q = z \leq 10$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
xline(a2, 'k--', 'a2', plt_setup{:});
xline(b2, 'k--', 'b2', plt_setup{:});
yline(0, 'k-');
k = k+1;

xlabel('Time (steps)');






% A = [1 1; 0 1];
% B = [.5;1];
% M = B;
% y = [2;0]; % Desired state
% for i = 1:49
% M = [A*M B];
% end
% u = M'*inv(M*M')*y;
