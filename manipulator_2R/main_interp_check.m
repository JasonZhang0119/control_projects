clear; clc; close all;

%% 参数设置
q0 = [0.0; 0.3];
qf = [pi/2; -pi/3];

q0_dot = [0.2; 0.0];
qf_dot = [0.0; -0.15];

q0_ddot = [0.0; 0.1];
qf_ddot = [0.0; 0.0];

t0 = 0.0;
tf = 3.0;

N = 301;
t_vec = linspace(t0, tf, N);

n = numel(q0);

q_cubic = zeros(n, N);
qd_cubic = zeros(n, N);
qdd_cubic = zeros(n, N);

q_quintic = zeros(n, N);
qd_quintic = zeros(n, N);
qdd_quintic = zeros(n, N);

%% 轨迹生成
for k = 1:N
    t = t_vec(k);

    [q_cubic(:, k), qd_cubic(:, k), qdd_cubic(:, k)] = ...
        CubicInterp(q0, qf, q0_dot, qf_dot, t0, tf, t);

    [q_quintic(:, k), qd_quintic(:, k), qdd_quintic(:, k)] = ...
        QuinticInterp(q0, qf, q0_dot, qf_dot, q0_ddot, qf_ddot, t0, tf, t);
end

%% 端点条件检查
tol = 1e-10;

[q_start_c, qd_start_c, ~] = CubicInterp(q0, qf, q0_dot, qf_dot, t0, tf, t0);
[q_end_c, qd_end_c, ~] = CubicInterp(q0, qf, q0_dot, qf_dot, t0, tf, tf);

if norm(q_start_c - q0) > tol || norm(qd_start_c - q0_dot) > tol || ...
   norm(q_end_c - qf) > tol || norm(qd_end_c - qf_dot) > tol
    error('main_interp_check:CubicBoundaryMismatch', ...
          'CubicInterp 端点位置或速度条件不满足。');
end

[q_start_q, qd_start_q, qdd_start_q] = ...
    QuinticInterp(q0, qf, q0_dot, qf_dot, q0_ddot, qf_ddot, t0, tf, t0);

[q_end_q, qd_end_q, qdd_end_q] = ...
    QuinticInterp(q0, qf, q0_dot, qf_dot, q0_ddot, qf_ddot, t0, tf, tf);

if norm(q_start_q - q0) > tol || norm(qd_start_q - q0_dot) > tol || ...
   norm(qdd_start_q - q0_ddot) > tol || ...
   norm(q_end_q - qf) > tol || norm(qd_end_q - qf_dot) > tol || ...
   norm(qdd_end_q - qf_ddot) > tol
    error('main_interp_check:QuinticBoundaryMismatch', ...
          'QuinticInterp 端点位置、速度或加速度条件不满足。');
end

disp('CubicInterp boundary check passed!');
disp('QuinticInterp boundary check passed!');

%% 绘图：Cubic
figure('Name', 'Cubic Interpolation Check');

labels_q = {'q_1', 'q_2'};

% ===== 位置 =====
subplot(3, 1, 1); hold on;
plot(t_vec, q_cubic, 'LineWidth', 1.5);

for i = 1:n
    plot(t0, q0(i), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(tf, qf(i), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
end

grid on;
ylabel('q');
title('Cubic - Position');
legend([labels_q, "start", "end"]);

% ===== 速度 =====
subplot(3, 1, 2); hold on;
plot(t_vec, qd_cubic, 'LineWidth', 1.5);

for i = 1:n
    plot(t0, q0_dot(i), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(tf, qf_dot(i), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
end

grid on;
ylabel('q dot');
title('Cubic - Velocity');
legend([labels_q, "start", "end"]);

% ===== 加速度 =====
subplot(3, 1, 3); hold on;
plot(t_vec, qdd_cubic, 'LineWidth', 1.5);

grid on;
xlabel('Time [s]');
ylabel('q ddot');
title('Cubic - Acceleration');
legend(labels_q);

%% 绘图：Quintic
figure('Name', 'Quintic Interpolation Check');

% ===== 位置 =====
subplot(3, 1, 1); hold on;
plot(t_vec, q_quintic, 'LineWidth', 1.5);

for i = 1:n
    plot(t0, q0(i), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(tf, qf(i), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
end

grid on;
ylabel('q');
title('Quintic - Position');
legend([labels_q, "start", "end"]);

% ===== 速度 =====
subplot(3, 1, 2); hold on;
plot(t_vec, qd_quintic, 'LineWidth', 1.5);

for i = 1:n
    plot(t0, q0_dot(i), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(tf, qf_dot(i), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
end

grid on;
ylabel('q dot');
title('Quintic - Velocity');
legend([labels_q, "start", "end"]);

% ===== 加速度 =====
subplot(3, 1, 3); hold on;
plot(t_vec, qdd_quintic, 'LineWidth', 1.5);

for i = 1:n
    plot(t0, q0_ddot(i), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(tf, qf_ddot(i), 's', 'MarkerSize', 8, 'LineWidth', 1.5);
end

grid on;
xlabel('Time [s]');
ylabel('q ddot');
title('Quintic - Acceleration');
legend([labels_q, "start", "end"]);