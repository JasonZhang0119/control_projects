%% main_admittance_control.m
% 测试 EndAdmittanceControl 的闭环脚本
%
% 控制链路：
%   外界力 Fext -> EndAdmittanceControl -> xd -> IK2R_Continuous
%             -> JointPositionPIDController -> 机器人动力学 -> 虚拟环境
%
% 说明：
% - 这个脚本用于测试“导纳控制输出末端期望位置 xd”的外环逻辑。
% - 内层使用关节位置控制器跟踪由逆解得到的关节参考。
% - EndAdmittanceControl 里的核心算法仍然是 TODO，
%   但这个脚本已经把完整接口和数据流搭好了。

clear;
clc;
close all;

%% ================================================================
% 1. 路径配置
% ================================================================
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root_dir, 'matlab')));

%% ================================================================
% 2. 机器人参数
% ================================================================
param = Param2R('standard');
n = numel(param.joint_types);

%% ================================================================
% 3. 仿真参数
% ================================================================
Ts = 0.001;
T_total = 6.0;
N = round(T_total / Ts);
t = (0:N-1) * Ts;

%% ================================================================
% 4. 初始位姿与虚拟墙
% ================================================================
q0 = deg2rad([30; -60]);
x0 = FK2R_Analytic(q0, param);

% 让初始末端位置稍微压入虚拟墙，方便观察导纳响应
x_wall = x0(1) - 0.03;
K_env = 2000;
D_env = 20;

%% ================================================================
% 5. 导纳控制器参数
% ================================================================
adm_param.Md = diag([2.0, 2.0]);
adm_param.Dd = diag([35.0, 35.0]);
adm_param.Kd = diag([80.0, 80.0]);

adm_options = struct();
adm_options.task_dim = 2;
adm_options.x0 = x0;
adm_options.xdot0 = zeros(2, 1);
adm_options.xddot0 = zeros(2, 1);

adm_controller = EndAdmittanceControl(adm_param, param, Ts, adm_options);

%% ================================================================
% 6. 内层位置控制器参数
% ================================================================
pos_pid_param.Kp = [180.0; 180.0];
pos_pid_param.Ki = [0.0; 0.0];
pos_pid_param.Kd = [16.0; 16.0];

pos_options = struct();
pos_options.tau_min = -100 * ones(n, 1);
pos_options.tau_max =  100 * ones(n, 1);

pos_controller = JointPositionPIDController(pos_pid_param, param, Ts, pos_options);

%% ================================================================
% 7. 初始状态
% ================================================================
q = zeros(n, N);
qd = zeros(n, N);
qdd = zeros(n, N);

q(:, 1) = q0;
qd(:, 1) = zeros(n, 1);
qdd(:, 1) = zeros(n, 1);

% 用于数值微分
xdot_prev = zeros(2, 1);

% 逆解连续分支选择
q_ref_prev = q0;

% 外环内部期望状态
xd_cmd = x0;
xdotd_cmd = zeros(2, 1);
xddotd_cmd = zeros(2, 1);

%% ================================================================
% 8. 数据记录
% ================================================================
x_log = zeros(2, N);
xdot_log = zeros(2, N);
xddot_log = zeros(2, N);

xd_cmd_log = zeros(2, N);
xdotd_cmd_log = zeros(2, N);
xddotd_cmd_log = zeros(2, N);

Fext_log = zeros(2, N);
tau_pos_log = zeros(n, N);
tau_env_log = zeros(n, N);
tau_total_log = zeros(n, N);
q_ref_log = zeros(n, N);
x_err_log = zeros(2, N);

%% ================================================================
% 9. 主循环
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
    % 虚拟墙接触模型
    % ------------------------------------------------------------
    penetration = x(1) - x_wall;
    Fext_on_robot = zeros(2, 1);

    if penetration > 0
        Fext_on_robot(1) = -K_env * penetration - D_env * xdot(1);
    end

    Fext_log(:, k) = Fext_on_robot;

    % ------------------------------------------------------------
    % 导纳控制器：Fext -> xd
    % ------------------------------------------------------------
    [xd_next, info_adm] = adm_controller.step( ...
        Fext_on_robot, ...
        x, ...
        xdot, ...
        xddot);

    xd_cmd = xd_next;
    xdotd_cmd = info_adm.xdotd;
    xddotd_cmd = info_adm.xddotd;

    xd_cmd_log(:, k) = xd_cmd;
    xdotd_cmd_log(:, k) = xdotd_cmd;
    xddotd_cmd_log(:, k) = xddotd_cmd;

    % ------------------------------------------------------------
    % 末端期望位置 -> 关节参考
    % ------------------------------------------------------------
    try
        q_ref = IK2R_Continuous(xd_cmd, param, q_ref_prev);
    catch ME
        warning('IK 失败，保留上一时刻关节参考：%s', ME.message);
        q_ref = q_ref_prev;
    end

    q_ref_prev = q_ref;
    q_ref_log(:, k) = q_ref;

    % ------------------------------------------------------------
    % 内层位置控制器：q_ref -> tau
    % ------------------------------------------------------------
    qd_ref = zeros(n, 1);
    qdd_ref = zeros(n, 1);
    tau_pos = pos_controller.step(q_ref, qd_ref, qdd_ref, q(:, k), qd(:, k));
    tau_pos_log(:, k) = tau_pos;

    % ------------------------------------------------------------
    % 环境力映射到关节空间
    % ------------------------------------------------------------
    tau_env = J' * Fext_on_robot;
    tau_env_log(:, k) = tau_env;

    % ------------------------------------------------------------
    % 机器人动力学
    % ------------------------------------------------------------
    [M, h, G] = Robot2R_Dynamics(q(:, k), qd(:, k), param);

    tau_total = tau_pos + tau_env;
    tau_total_log(:, k) = tau_total;

    qdd_current = M \ (tau_total - h - G);

    qd(:, k+1) = qd(:, k) + qdd_current * Ts;
    q(:, k+1)  = q(:, k)  + qd(:, k+1) * Ts;
    qdd(:, k+1) = qdd_current;

    x_err_log(:, k) = xd_cmd - x;
end

%% ================================================================
% 10. 最后一个采样点补齐
% ================================================================
x_log(:, N) = FK2R_Analytic(q(:, N), param);
J = Jacobian2R_Analytic(q(:, N), param);
xdot_log(:, N) = J * qd(:, N);
if N > 1
    xddot_log(:, N) = (xdot_log(:, N) - xdot_log(:, N-1)) / Ts;
end

xd_cmd_log(:, N) = xd_cmd;
xdotd_cmd_log(:, N) = xdotd_cmd;
xddotd_cmd_log(:, N) = xddotd_cmd;
q_ref_log(:, N) = q_ref_prev;

Fext_log(:, N) = Fext_log(:, N-1);
tau_pos_log(:, N) = tau_pos_log(:, N-1);
tau_env_log(:, N) = tau_env_log(:, N-1);
tau_total_log(:, N) = tau_total_log(:, N-1);
x_err_log(:, N) = xd_cmd - x_log(:, N);

%% ================================================================
% 11. 绘图
% ================================================================
figure('Name', 'End Admittance Control Test', 'Color', 'w');

subplot(2, 2, 1);
plot(t, Fext_log(1, :), 'LineWidth', 1.2);
title('x 方向外界力');
xlabel('时间 [s]');
ylabel('力 [N]');
grid on;

subplot(2, 2, 2);
plot(t, x_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, xd_cmd_log(1, :), '--', 'LineWidth', 1.2);
yline(x_wall, ':', 'LineWidth', 1.2);
title('末端 x 方向位置');
xlabel('时间 [s]');
ylabel('x [m]');
legend('x', 'x_d', 'wall');
grid on;

subplot(2, 2, 3);
plot(t, tau_pos_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, tau_pos_log(2, :), 'LineWidth', 1.2);
title('位置控制器输出力矩');
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

figure('Name', 'End Admittance Control Details', 'Color', 'w');

subplot(2, 2, 1);
plot(t, xd_cmd_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, x_log(1, :), 'LineWidth', 1.2);
title('x 方向位置跟踪');
xlabel('时间 [s]');
ylabel('位置 [m]');
legend('x_d', 'x');
grid on;

subplot(2, 2, 2);
plot(t, x_err_log(1, :), 'LineWidth', 1.2);
title('x 方向位置误差');
xlabel('时间 [s]');
ylabel('误差 [m]');
grid on;

subplot(2, 2, 3);
plot(t, xdot_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, xdotd_cmd_log(1, :), '--', 'LineWidth', 1.2);
title('x 方向速度');
xlabel('时间 [s]');
ylabel('速度 [m/s]');
legend('xdot', 'xdot_d');
grid on;

subplot(2, 2, 4);
plot(t, q_ref_log(1, :), 'LineWidth', 1.2);
hold on;
plot(t, q_ref_log(2, :), 'LineWidth', 1.2);
title('关节参考轨迹');
xlabel('时间 [s]');
ylabel('关节角 [rad]');
legend('q_{ref,1}', 'q_{ref,2}');
grid on;
