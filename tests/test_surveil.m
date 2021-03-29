clear all
close all

params.Nsteps = 61;
params.N = params.Nsteps-1;
params.agents = 2;

constraints = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control variable
u = sdpvar(params.N, 2*params.agents); %inputs, a variable
u_max = 20;
u_bds = -u_max*ones(params.N, 2*params.agents) <= u <= u_max*ones(params.N, 2*params.agents);
constraints = [constraints; u_bds];

%% Dynamics
x01 = [11, 0, 11, 0];
x02 = [4, 0, 4, 0];
x = [x01, x02];
Ts = 0.1;
A = kron(eye(2*params.agents), [1 Ts; 0 1]);
B = kron(eye(2*params.agents), [Ts*Ts./2; Ts]);
for k = 1:params.N
    x_k = x(end, :)';
    u_k = u(k, :)';
   
    x_kp1 = A*x_k + B*u_k;
    x = [x; x_kp1'];
end

% v_max = 4;
%vx_bds = -v_max*ones(params.Nsteps, 1) <= x(:, 2) <= v_max*ones(params.Nsteps, 1);
% vy_bds = -v_max*ones(params.Nsteps, 1) <= x(:, 4) <= v_max*ones(params.Nsteps, 1);
% constraints = [constraints; vy_bds];
%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_lb1 = 5;
x_ub1 = 10;
y_lb1 = 5;
y_ub1 = 10;

x1 = [x(:, 1), x(:, 3)];
x2 = [x(:, 5), x(:, 7)];

flag_lite = true;
if flag_lite
    [constr1, theta1, zg1] = theta_goal_lite(x1, x_lb1, x_ub1, y_lb1, y_ub1);
    [constr2, theta2, zg2] = theta_goal_lite(x2, x_lb1, x_ub1, y_lb1, y_ub1);
else
    [constr1, theta1] = theta_goal(x1, x_lb1, x_ub1, y_lb1, y_ub1);
    [constr2, theta2] = theta_goal(x2, x_lb1, x_ub1, y_lb1, y_ub1);
end
constraints = [constraints; constr1; constr2];
%constraints = [constraints; constr1];
%
theta_goal = sdpvar(params.Nsteps, 1);
for t=1:params.Nsteps
    [c, theta_goal(t)] = quant_or([theta1(t); theta2(t)]);
    constraints = [constraints; c];
end

a1 = 0;
b1 = 20;
[constr, theta_finally1] = quant_finally(theta_goal, 1, a1, b1);
constraints = [constraints; constr];

a2 = 20;
b2 = 40;
[constr, theta_finally2] = quant_finally(theta_goal, 1, a2, b2);
constraints = [constraints; constr];

[constr, theta_f] = quant_and([theta_finally1; theta_finally2]);
constraints = [constraints; constr];


%% CHARGE 1
x_lb_ch1 = 11;
x_ub_ch1 = 12;
y_lb_ch1 = 9;
y_ub_ch1 = 10;
[constr, z_1, z_ch1] = theta_goal_lite(x1, x_lb_ch1, x_ub_ch1, y_lb_ch1, y_ub_ch1);
constraints = [constraints; constr];


% [constr, theta_ch1_globally] = quant_finally(z_1, 1, 0, 20);
% constraints = [constraints; constr];
% 
% [constr, theta_ch0_globally] = quant_finally(z_1, 1, 40, 55);
% constraints = [constraints; constr];

a_ch = 0;
b_ch = 20;
a_life = 0;
b_life = 20;
z_g = sdpvar(params.Nsteps-b_ch, 1);
for t = 1:params.Nsteps-b_ch
    [constr, z_g(t)] = quant_globally(z_1, t, a_ch, b_ch);
    constraints = [constraints; constr];
end

z_f = sdpvar(params.Nsteps-b_life-b_ch, 1);
for t = 1:params.Nsteps-b_life-b_ch
    [constr, z_f(t)] = quant_finally(z_g, t, a_life, b_life);
    constraints = [constraints; constr];
end

[constr, z_gg] = quant_globally(z_f, 1, 0, params.Nsteps-b_ch-b_life-1);
constraints = [constraints; constr];

% [constr, theta_ch1_globally2] = quant_finally(theta_ch1, 1, 31, 60);
% constraints = [constraints; constr];


%% CHARGE 2
x_lb_ch2 = 3;
x_ub_ch2 = 4;
y_lb_ch2 = 5;
y_ub_ch2 = 6;
[constr, theta_ch2, z_ch2] = theta_goal_lite(x2, x_lb_ch2, x_ub_ch2, y_lb_ch2, y_ub_ch2);
constraints = [constraints; constr];

[constr, theta_ch2_globally] = quant_finally(theta_ch2, 1, 0, 20);
constraints = [constraints; constr];

[constr, theta_ch3_globally] = quant_finally(theta_ch2, 1, 40, 55);
constraints = [constraints; constr];
% 
% %%%%%%%%%%%
[constr, theta_phi] = quant_and([theta_f; theta_ch2_globally; theta_ch3_globally; z_gg]);
constraints = [constraints; constr]; %theta_phi(1)==5  theta_phi(1)>=1

%theta_phi = theta_ch1_globally;

objective = theta_phi(1);
%objective = L-sum(abs(u));

options = sdpsettings('solver','gurobi');
sol = optimize(constraints, -objective, options);

%%
R = value(theta_phi);
fprintf('Right time rob: %i(time units)\n', R);


%%%%%%%%%%%%%%%%%%%%
color_red = [200./255, 0, 0];
color_blue = [51./255, 102./255, 1];
color_green = [.0 .875 .0];
color_orange = [1 102./255 0];


figure
N_plot = params.Nsteps;
hold on
p1 = plot(value(x(1:21, 1)), value(x(1:21, 3)), '-', 'color', color_blue, 'linewidth', 3);
plot(value(x(21:N_plot, 1)), value(x(21:N_plot, 3)), '-', 'color', color_blue, 'linewidth', 2);
%p1 = plot(value(x(1:N_plot, 1)), value(x(1:N_plot, 3)), '-', 'color', color_blue, 'linewidth', 3);
scatter(value(x(1:21, 1)), value(x(1:21, 3)), 30, color_blue, 'filled');
scatter(value(x(21:N_plot, 1)), value(x(21:N_plot, 3)), 20, color_blue, 'filled');
%scatter(value(x(1+(a1:b1+floor(R)), 1)), value(x(1+(a1:b1+floor(R)), 3)), 30, 'r', 'filled');
%scatter(value(x(1+(a1:b1), 1)), value(x(1+(a1:b1), 3)), 30, 'r', 'filled');
%scatter(value(x(1+(a2:b2), 1)), value(x(1+(a2:b2), 3)), 30, color_blue, 'filled');
scatter(x(1, 1), x(1, 3), 500, 'filled', 'p', 'MarkerFaceColor', color_blue);

p2 = plot(value(x(24:43, 5)), value(x(24:43, 7)), '-', 'color', color_orange, 'linewidth', 3);
plot(value(x(1:24, 5)), value(x(1:24, 7)), '-', 'color', color_orange, 'linewidth', 2);
plot(value(x(43:N_plot, 5)), value(x(43:N_plot, 7)), '-', 'color', color_orange, 'linewidth', 2);

scatter(value(x(25:42, 5)), value(x(25:42, 7)), 30, color_orange, 'filled');
scatter(value(x(1:24, 5)), value(x(1:24, 7)), 20, color_orange, 'filled');
scatter(value(x(43:N_plot, 5)), value(x(43:N_plot, 7)), 20, color_orange, 'filled');
%scatter(value(x(1+(a1:b1), 5)), value(x(1+(a1:b1), 7)), 30, color_orange, 'filled');
%scatter(value(x(1+(a2:a2+floor(R)), 5)), value(x(1+(a2:a2+floor(R)), 7)), 30, 'r', 'filled');
%scatter(value(x(1+(a2:b2), 5)), value(x(1+(a2:b2), 7)), 30, 'r', 'filled');
scatter(x(1, 5), x(1, 7), 500, 'filled', 'p', 'MarkerFaceColor', color_orange);

hf1 = fill_between([x_lb1,x_ub1],y_lb1*ones(1,2), y_ub1*ones(1,2));
hf1.FaceColor = color_green;
alpha(hf1, .2);

h_ch1 = fill_between([x_lb_ch1,x_ub_ch1],y_lb_ch1*ones(1,2), y_ub_ch1*ones(1,2));
h_ch1.FaceColor = 'y';
alpha(h_ch1, .6);


h_ch2 = fill_between([x_lb_ch2,x_ub_ch2],y_lb_ch2*ones(1,2), y_ub_ch2*ones(1,2));
h_ch2.FaceColor = 'y';
alpha(h_ch2, .6);
legend([p1, p2], {'Agent 1', 'Agent 2'}, 'AutoUpdate', 'off', 'Location','southeast', 'FontSize',15); 
axis equal;
xlabel('X', 'fontsize', 15);
ylabel('Y', 'fontsize', 15);
xlim([2.5,12.5]);
ylim([3.4,11.5]);
text(7,9.5,'Goal', 'fontsize', 15);
text(10.3,8.3,'Charge^{(1)}', 'fontsize', 15);
text(2.6,6.5,'Charge^{(2)}', 'fontsize', 15);
text(11.5,11,'\alpha_1', 'fontsize', 20);
text(4.5,4,'\alpha_2', 'fontsize', 20);

hold on
for t = 1:N_plot
    if exist('plot_vehicle1', 'var')
        delete(plot_vehicle1);
        delete(plot_vehicle2);
    end
    plot_vehicle1 = scatter(x(t, 1), x(t, 3), 200, 'filled', 'p', 'MarkerFaceColor', color_green);
    plot_vehicle2 = scatter(x(t, 5), x(t, 7), 200, 'filled', 'p', 'MarkerFaceColor', color_green);
    pause(0.2);
end

%%
%%
hh = figure;
plt_label = {'fontsize', 12}; %'FontWeight','bold'}
plt_setup = {'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'right'};
subplot(2, 4, [1,2]);
hold on
plot(0:(params.Nsteps-1), value(x(:, 1)), '-', 'color', color_blue, 'linewidth', 2);
plot(0:(params.Nsteps-1), value(x(:, 5)), '-.', 'color', color_orange, 'linewidth', 2);

TT1 = find(value(zg1)');
scatter(TT1-1, value(x(TT1, 1)), 50, color_green, 'filled');

TT2 = find(value(zg2)');
scatter(TT2-1, value(x(TT2, 5)), 50, color_green, 'filled');

%scatter(TT-1, value(x(TT, 3)), 30, 'b', 'filled');

xline(a1, 'k:');
xline(b1, 'k:');
xline(b2, 'k:');
yline(x_lb1, ':', 'color', 'k', 'linewidth', 0.5);
yline(x_ub1, ':', 'color', 'k', 'linewidth', 0.5);

yline(x_lb_ch1, ':', 'color', 'k', 'linewidth', 0.5);
yline(x_ub_ch1, ':', 'color', 'k', 'linewidth', 0.5);

yline(x_lb_ch2, ':', 'color', 'k', 'linewidth', 0.5);
yline(x_ub_ch2, ':', 'color', 'k', 'linewidth', 0.5);

plot_rect(a1, b1, x_lb1, x_ub1, color_green, 0.2);
plot_rect(a2, b2, x_lb1, x_ub1, color_green, 0.2);
%plot_rect(b1, b1+R, x_lb1, x_ub1, 'y', 0.5);

plot_rect(0, params.Nsteps-1, x_lb_ch1, x_ub_ch1, 'y', 0.4);
plot_rect(0, params.Nsteps-1, x_lb_ch2, x_ub_ch2, 'y', 0.4);

xlim([0, params.Nsteps-1]);
xlabel('Time (steps)', plt_label{:});
ylabel('X-Position', plt_label{:});
title('(a) Agent Positions', 'HorizontalAlignment','center','EdgeColor','none','FontSize',18);
%----------------------------------------------------------
subplot(2, 4, [5,6]);
hold on
plot(0:(params.Nsteps-1), value(x(:, 3)), '-', 'color', color_blue, 'linewidth', 2);
plot(0:(params.Nsteps-1), value(x(:, 7)), '-.', 'color', color_orange, 'linewidth', 2);
xline(a1, 'k:');
xline(b1, 'k:');
xline(b2, 'k:');
yline(y_lb1, ':', 'color', 'k', 'linewidth', 0.5);
yline(y_ub1, ':', 'color', 'k', 'linewidth', 0.5);

yline(y_lb_ch1, ':', 'color', 'k', 'linewidth', 0.5);
yline(y_ub_ch1, ':', 'color', 'k', 'linewidth', 0.5);

yline(y_lb_ch2, ':', 'color', 'k', 'linewidth', 0.5);
yline(y_ub_ch2, ':', 'color', 'k', 'linewidth', 0.5);

TT1 = find(value(zg1)');
scatter(TT1-1, value(x(TT1, 3)), 50, color_green, 'filled');

TT2 = find(value(zg2)');
scatter(TT2-1, value(x(TT2, 7)), 50, color_green, 'filled');

plot_rect(a1, b1, y_lb1, y_ub1, color_green, 0.2);
plot_rect(a2, b2, y_lb1, y_ub1, color_green, 0.2);
%plot_rect(b1, b1+R, x_lb1, x_ub1, 'y', 0.5);

plot_rect(0, params.Nsteps-1, y_lb_ch1, y_ub_ch1, 'y', 0.4);
plot_rect(0, params.Nsteps-1, y_lb_ch2, y_ub_ch2, 'y', 0.4);

xlim([0, params.Nsteps-1]);
xlabel('Time (steps)', plt_label{:});
ylabel('Y-Position', plt_label{:});

subplot(2, 4, 3);
hold on
plot(0:(params.Nsteps-1), value(x(:, 2)), '-', 'color', color_blue, 'linewidth', 2);
plot(0:(params.Nsteps-1), value(x(:, 6)), '-.', 'color', color_orange, 'linewidth', 2);
xlabel('Time (steps)', plt_label{:});
ylabel('X-Velocity', plt_label{:});
title('(b) Agent Velocities', 'HorizontalAlignment','center','EdgeColor','none','FontSize',18);

subplot(2, 4, 7);
hold on
plot(0:(params.Nsteps-1), value(x(:, 4)), '-', 'color', color_blue, 'linewidth', 2);
plot(0:(params.Nsteps-1), value(x(:, 8)), '-.', 'color', color_orange, 'linewidth', 2);
xlabel('Time (steps)', plt_label{:});
ylabel('Y-Velocity', plt_label{:});

subplot(2, 4, 4);
hold on
plot(0:(params.N-1), value(u(:, 1)), '-', 'color', color_blue, 'linewidth', 1);
plot(0:(params.N-1), value(u(:, 3)), '-.', 'color', color_orange, 'linewidth', 1);
yline(u_max, '--', 'u_{max}', 'color', 'k', 'linewidth', 1, 'LabelVerticalAlignment', 'top');
yline(-u_max, '--', 'u_{min}', 'color', 'k', 'linewidth', 1, 'LabelVerticalAlignment', 'bottom');
ylim([-1.3*u_max, 1.3*u_max]);
xlabel('Time (steps)', plt_label{:});
ylabel('X-Input', plt_label{:});
title('(c) Control Inputs', 'HorizontalAlignment','center','EdgeColor','none','FontSize',18);
set(gca, 'box', 'off')

subplot(2, 4, 8);
hold on
plot(0:(params.N-1), value(u(:, 2)), '-', 'color', color_blue, 'linewidth', 1);
plot(0:(params.N-1), value(u(:, 4)), '-.', 'color', color_orange, 'linewidth', 1);
yline(u_max, '--', 'u_{max}', 'color', 'k', 'linewidth', 1, 'LabelVerticalAlignment', 'top');
yline(-u_max, '--', 'u_{min}', 'color', 'k', 'linewidth', 1, 'LabelVerticalAlignment', 'bottom');
ylim([-1.3*u_max, 1.3*u_max]);
xlabel('Time (steps)', plt_label{:});
ylabel('Y-Input', plt_label{:});
set(gca, 'box', 'off')

set(gcf, 'PaperType', 'uslegal', 'PaperOrientation', 'Landscape');
%set(hh,'Position',[-1.48 2.22, 16.96, 4.27]);
%print(hh,'ex_agents', '-bestfit', '-dpdf') % then print it

%%
z_or = max([value(zg1), value(zg2)], [], 2);
fz = 15;
plt_setup = {'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'right'};

figure;
n_col = 3;
n_row = 2;
k = 1;
hrz = params.Nsteps;

subplot(n_row, n_col, k);
hold on
plot_chi(z_or, color_red);
%plot(0:size(z_or, 1)-1, z_or, 'color', color_red, 'linewidth', 2);
title('(a)$$\quad\chi_{goal}(\mathbf{x}), \ \ goal = [\alpha_1 \in Goal \vee \alpha_2\in Goal]$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
%xlabel('Time (steps)');
xline(a1, 'k--');
xline(b1, 'k--');
xline(a2, 'k--');
xline(b2, 'k--');
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_chi(z_ch1, color_red);
%plot(0:size(z_ch1, 1)-1, value(z_ch1), 'color', color_red, 'linewidth', 2);
title('(b)$$\quad\chi_{ch_1}(\mathbf{x}), \ \ ch_1= [\alpha_1 \in Charge^{(1)}]$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_chi(z_ch2, color_red);
%plot(0:size(z_ch2, 1)-1, value(z_ch2), 'color', color_red, 'linewidth', 2);
title('(c)$$\quad\chi_{ch_2}(\mathbf{x}), \ \ ch_2=[\alpha_2 \in Charge^{(2)}]$$', 'interpreter','latex', 'FontSize', fz);
xlim([0, hrz]);
%xlabel('Time (steps)');
xline(0, 'k--');
xline(20, 'k--');
xline(40, 'k--');
xline(55, 'k--');
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_theta(theta_goal, color_red);

%plot(0:size(theta_goal, 1)-1, value(theta_goal), 'color', color_red, 'linewidth', 2);
title('(d)$$\quad\theta^+_{goal}(\mathbf{x}),\ \  goal = [\alpha_1 \in Goal \vee \alpha_2\in Goal]$$', 'interpreter','latex', 'FontSize', fz);

xlim([0, hrz]);
xline(a1, 'k--');
xline(b1, 'k--');
xline(a2, 'k--');
xline(b2, 'k--');
yline(0, 'k-');
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_theta(z_1, color_red);
%plot(0:size(z_1, 1)-1, value(z_1), 'color', color_red, 'linewidth', 2);
title('(e)$$\quad\theta^+_{ch_1}(\mathbf{x}),\ \  ch_1 = [\alpha_1 \in Charge^{(1)}]$$', 'interpreter','latex', 'FontSize', fz);

xlim([0, hrz]);
yline(0, 'k-');
k = k+1;

subplot(n_row, n_col, k);
hold on
plot_theta(theta_ch2, color_red);
%plot(0:size(theta_ch2, 1)-1, value(theta_ch2), 'color', color_red, 'linewidth', 2);
title('(f)$$\quad\theta^+_{ch_2}(\mathbf{x}),\ \  ch_2 = [\alpha_2 \in Charge^{(2)}]$$', 'interpreter','latex', 'FontSize', fz);

xlim([0, hrz]);
xline(0, 'k--');
xline(20, 'k--');
xline(40, 'k--');
xline(55, 'k--');
yline(0, 'k-');
k = k+1;

set(gcf, 'PaperType', 'uslegal', 'PaperOrientation', 'Landscape');
%print(gcf,'ex_agents', '-bestfit', '-dpdf') % then print it


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%