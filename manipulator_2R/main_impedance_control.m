%% main_impedance_control.m
% 测试 EndImpedanceControl 的闭环脚本
%
% 控制链路：
%   末端实际状态 (x, xdot, xddot)
%       -> EndImpedanceControl
%       -> 任务空间力指令 Fcmd
%       -> JointForcePIDController
%       -> 关节力矩 tau
%       -> 机器人动力学
%       -> 虚拟环境 / 接触力反馈
%
% 说明：
% - 这个脚本用于测试“阻抗控制输出任务空间力指令 Fcmd”的外环逻辑。
% - 内层使用 force control controller 跟踪该 Fcmd。

clear;
clc;
close all;

%% ================================================================
% 路径配置
% ================================================================
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root_dir, 'matlab')));

%% ================================================================
% 机器人参数
% ================================================================
param = Param2R('standard');
n = numel(param.joint_types);

%% ================================================================
% 仿真参数
% ================================================================
Ts = 0.001;
T_total = 6.0;
N = round(T_total / Ts);
t = (0:N-1) * Ts;


%% ================================================================
% 末端期望轨迹
% ================================================================
q0 = deg2rad([30; -60]);
x0 = FK2R_Analytic(q0, param);

x_ref = repmat(x0, 1, N);
x_ref(1, :) = x0(1);
x_ref(2, :) = x0(2);

xdot_ref = zeros(2, N);
xddot_ref = zeros(2, N);

F_env_list = zeros(2, N);
F_env_list(1, t>=1) = 1;

%% ================================================================
% 阻抗控制器参数
% ================================================================
imp_param.Md = diag([1.0, 1.0]);
imp_param.Dd = diag([1.0, 1.0]);
imp_param.Kd = diag([1.0, 1.0]);

imp_options = struct();
imp_options.task_dim = 2;
imp_options.x0 = x0;
imp_options.x_dot0 = zeros(2, 1);
imp_options.x_ddot0 = zeros(2, 1);

imp_controller = EndImpedanceControl(imp_param, param, Ts, imp_options);

%% ================================================================
% 内层 force control 参数
% ================================================================
force_pid_param.Kp = [1.0; 1.0];
force_pid_param.Ki = [0.0; 0.0];
force_pid_param.Kd = [0.0; 0.0];

force_options = struct();
force_options.task_dim = 2;
force_options.tau_min = -100 * ones(n, 1);
force_options.tau_max =  100 * ones(n, 1);

force_controller = JointForcePIDController(force_pid_param, param, Ts, force_options);

%% ================================================================
% 8. 初始状态
% ================================================================
q = zeros(n, N);
qd = zeros(n, N);
qdd = zeros(n, N);

q(:, 1) = q0;
qd(:, 1) = zeros(n, 1);
qdd(:, 1) = zeros(n, 1);

% 用于数值微分
xdot_prev = zeros(2, 1);

%% ================================================================
% 9. 数据记录
% ================================================================
x_log = zeros(2, N);
xdot_log = zeros(2, N);
xddot_log = zeros(2, N);

x_ref_log = zeros(2, N);
xdot_ref_log = zeros(2, N);
xddot_ref_log = zeros(2, N);

Fcmd_log = zeros(2, N);
F_meas_log = zeros(2, N);
F_env_log = zeros(2, N);

tau_ctrl_log = zeros(n, N);
tau_env_log = zeros(n, N);
tau_total_log = zeros(n, N);

force_err_log = zeros(2, N);
x_err_log = zeros(2, N);

%% ================================================================
% 10. 主循环
% ================================================================
for k = 1:N-1

    % ------------------------------------------------------------
    % 当前末端状态
    % ------------------------------------------------------------
    x = FK2R_Analytic(q(:, k), param);
    J = Jacobian2R_Analytic(q(:, k), param);
    xdot = J * qd(:, k);

    if k == 1
        xddot = zeros(2, 1);
    else
        xddot = (xdot - xdot_prev) / Ts;
    end

    xdot_prev = xdot;

    x_log(:, k) = x;
    xdot_log(:, k) = xdot;
    xddot_log(:, k) = xddot;

    % ------------------------------------------------------------
    % 参考轨迹
    % ------------------------------------------------------------
    x_ref_k = x_ref(:, k);
    xdot_ref_k = xdot_ref(:, k);
    xddot_ref_k = xddot_ref(:, k);

    x_ref_log(:, k) = x_ref_k;
    xdot_ref_log(:, k) = xdot_ref_k;
    xddot_ref_log(:, k) = xddot_ref_k;

    % ------------------------------------------------------------
    % 阻抗控制器：x / xd -> Fcmd
    % ------------------------------------------------------------
    [Fcmd, info_imp] = imp_controller.step( ...
        x, ...
        xdot, ...
        xddot, ...
        x_ref_k, ...
        xdot_ref_k, ...
        xddot_ref_k);

    Fcmd_log(:, k) = Fcmd;

    % ------------------------------------------------------------
    % 虚拟墙接触模型
    % ------------------------------------------------------------
    F_env_on_robot = F_env_list(:, k);

    % 力传感器测得的是机器人对环境的作用力
    F_meas = -F_env_on_robot

    F_env_log(:, k) = F_env_on_robot;
    F_meas_log(:, k) = F_meas;

    % ------------------------------------------------------------
    % 内层 force control：Fcmd -> tau
    % ------------------------------------------------------------
    tau_ctrl = force_controller.step( ...
        Fcmd, ...
        F_meas, ...
        q(:, k), ...
        qd(:, k), ...
        qdd(:, k));

    tau_ctrl_log(:, k) = tau_ctrl;

    % ------------------------------------------------------------
    % 环境力映射到关节空间
    % ------------------------------------------------------------
    tau_env = J' * F_env_on_robot;
    tau_env_log(:, k) = tau_env;

    % ------------------------------------------------------------
    % 机器人动力学
    % ------------------------------------------------------------
    [M, h, G] = Robot2R_Dynamics(q(:, k), qd(:, k), param);

    tau_total = tau_ctrl + tau_env;
    tau_total_log(:, k) = tau_total;

    qdd_current = M \ (tau_total - h - G);

    qd(:, k+1) = qd(:, k) + qdd_current * Ts;
    q(:, k+1)  = q(:, k)  + qd(:, k+1) * Ts;
    qdd(:, k+1) = qdd_current;

    force_err_log(:, k) = Fcmd - F_meas;
    x_err_log(:, k) = x_ref_k - x;
end

%% ================================================================
% 11. 最后一个采样点补齐
% ================================================================
x_log(:, N) = FK2R_Analytic(q(:, N), param);
J = Jacobian2R_Analytic(q(:, N), param);
xdot_log(:, N) = J * qd(:, N);
if N > 1
    xddot_log(:, N) = (xdot_log(:, N) - xdot_log(:, N-1)) / Ts;
end

x_ref_log(:, N) = x_ref(:, N);
xdot_ref_log(:, N) = xdot_ref(:, N);
xddot_ref_log(:, N) = xddot_ref(:, N);

Fcmd_log(:, N) = Fcmd_log(:, N-1);
F_meas_log(:, N) = F_meas_log(:, N-1);
F_env_log(:, N) = F_env_log(:, N-1);
tau_ctrl_log(:, N) = tau_ctrl_log(:, N-1);
tau_env_log(:, N) = tau_env_log(:, N-1);
tau_total_log(:, N) = tau_total_log(:, N-1);
force_err_log(:, N) = force_err_log(:, N-1);
x_err_log(:, N) = x_err_log(:, N-1);

%% ================================================================
% 12. 绘图
% ================================================================
figure('Name', 'End Impedance Control Test', 'Color', 'w');

subplot(2, 2, 1);
plot(t, Fcmd_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, F_meas_log(1, :), 'LineWidth', 1.2);
title('x 方向力跟踪');
xlabel('时间 [s]');
ylabel('力 [N]');
legend('Fcmd_x', 'F_meas_x');
grid on;

subplot(2, 2, 2);
plot(t, x_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, x_ref_log(1, :), '--', 'LineWidth', 1.2);
title('末端 x 方向位置');
xlabel('时间 [s]');
ylabel('x [m]');
legend('x', 'x_{ref}', 'wall');
grid on;

subplot(2, 2, 3);
plot(t, tau_ctrl_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, tau_ctrl_log(2, :), 'LineWidth', 1.2);
title('力控制器输出力矩');
xlabel('时间 [s]');
ylabel('\tau [Nm]');
legend('\tau_1', '\tau_2');
grid on;

subplot(2, 2, 4);
plot(t, rad2deg(q(1, :)), 'LineWidth', 1.2);
hold on;
plot(t, rad2deg(q(2, :)), 'LineWidth', 1.2);
title('关节角');
xlabel('时间 [s]');
ylabel('角度 [deg]');
legend('q_1', 'q_2');
grid on;

figure('Name', 'End Impedance Control Details', 'Color', 'w');

subplot(2, 2, 1);
plot(t, x_ref_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, x_log(1, :), 'LineWidth', 1.2);
title('x 方向位置跟踪');
xlabel('时间 [s]');
ylabel('位置 [m]');
legend('x_{ref}', 'x');
grid on;

subplot(2, 2, 2);
plot(t, force_err_log(1, :), 'LineWidth', 1.2);
title('x 方向力误差');
xlabel('时间 [s]');
ylabel('误差 [N]');
grid on;

subplot(2, 2, 3);
plot(t, xdot_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, xdot_ref_log(1, :), '--', 'LineWidth', 1.2);
title('x 方向速度');
xlabel('时间 [s]');
ylabel('速度 [m/s]');
legend('xdot', 'xdot_{ref}');
grid on;

subplot(2, 2, 4);
plot(t, qdd(1, :), 'LineWidth', 1.2);
hold on;
plot(t, qdd(2, :), 'LineWidth', 1.2);
title('关节加速度');
xlabel('时间 [s]');
ylabel('加速度 [rad/s^2]');
legend('qdd_1', 'qdd_2');
grid on;
