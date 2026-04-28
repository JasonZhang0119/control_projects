%% main_check_joint_force_pid_controller.m
%MAIN_CHECK_JOINT_FORCE_PID_CONTROLLER 测试关节力控器
%
% Notes
% -----
% - 使用虚拟墙环境测试 force control。
% - 末端与竖直墙接触，控制 x 方向接触力。
% - 环境力通过 J' 映射到关节空间进入机器人动力学。

clear;
clc;
close all;

%% ================================================================
% 1. 创建机器人参数
% ================================================================
param = Param2R('standard');
n = numel(param.joint_types);

%% ================================================================
% 2. 仿真参数
% ================================================================
Ts = 0.001;
T_total = 5.0;
N = round(T_total / Ts);
t = (0:N-1) * Ts;

%% ================================================================
% 3. 虚拟墙环境
% ================================================================
x_wall = 1.50;       % 竖直墙位置
K_env = 2000;        % 墙刚度
D_env = 20;          % 墙阻尼

%% ================================================================
% 4. 力控目标
% ================================================================
F_ref = zeros(2, N);
F_ref(1, :) = 5.0;   % 期望机器人对墙施加 +x 方向 5 N
F_ref(2, :) = 0.0;

%% ================================================================
% 5. 控制器参数
% ================================================================
force_pid_param.Kp = [2.0; 0.0];
force_pid_param.Ki = [5.0; 0.0];
force_pid_param.Kd = [0.0; 0.0];

options = struct();
options.task_dim = 2;
options.tau_min = [-100; -100];
options.tau_max = [ 100;  100];
options.use_gravity_compensation = true;
options.use_coriolis_compensation = true;

controller = JointForcePIDController(force_pid_param, param, Ts, options);

%% ================================================================
% 6. 初始状态
% ================================================================
q = zeros(n, N);
qd = zeros(n, N);
qdd = zeros(n, N);

q(:,1) = deg2rad([30; -60]);
qd(:,1) = zeros(n,1);

%% ================================================================
% 7. 数据记录
% ================================================================
x_log = zeros(2, N);
xd_log = zeros(2, N);

F_meas_log = zeros(2, N);
F_env_log = zeros(2, N);

tau_ctrl_log = zeros(n, N);
tau_env_log = zeros(n, N);
tau_total_log = zeros(n, N);

%% ================================================================
% 8. 主仿真循环
% ================================================================
for k = 1:N-1

    % 当前末端状态
    x =  FK2R_Analytic(q(:,k), param);
    J = Jacobian2R_Analytic(q(:,k), param);
    xd = J * qd(:,k);

    x_log(:,k) = x;
    xd_log(:,k) = xd;

    % ============================================================
    % 虚拟墙接触模型
    % ============================================================
    penetration = x(1) - x_wall;

    F_env_on_robot = zeros(2,1);

    if penetration > 0
        F_env_on_robot(1) = -K_env * penetration - D_env * xd(1);
    end

    % 力传感器测得机器人对环境的力
    F_meas = -F_env_on_robot;

    F_env_log(:,k) = F_env_on_robot;
    F_meas_log(:,k) = F_meas;

    % ============================================================
    % 力控制器
    % ============================================================
    [tau_ctrl, info] = controller.step( ...
        F_ref(:,k), ...
        F_meas, ...
        q(:,k), ...
        qd(:,k),...
        qdd(:, k));

    tau_ctrl_log(:,k) = tau_ctrl;

    % 环境外力映射到关节空间
    tau_env = J' * F_env_on_robot;
    tau_env_log(:,k) = tau_env;

    % 机器人动力学
    [M, h, G] = Robot2R_Dynamics(q(:,k), qd(:,k), param);

    tau_total = tau_ctrl + tau_env;
    tau_total_log(:,k) = tau_total;

    qdd_current = M \ (tau_total - h - G);

    qd(:,k+1) = qd(:,k) + qdd_current * Ts;
    q(:,k+1)  = q(:,k)  + qd(:,k+1) * Ts;
    qdd(:,k+1) = qdd_current;
end

%% ================================================================
% 9. 补最后一个采样点
% ================================================================
x_log(:,N) =  FK2R_Analytic(q(:,N), param);
J = Jacobian2R_Analytic(q(:,N), param);
xd_log(:,N) = J * qd(:,N);

F_meas_log(:,N) = F_meas_log(:,N-1);
F_env_log(:,N) = F_env_log(:,N-1);
tau_ctrl_log(:,N) = tau_ctrl_log(:,N-1);
tau_env_log(:,N) = tau_env_log(:,N-1);
tau_total_log(:,N) = tau_total_log(:,N-1);

%% ================================================================
% 10. 绘图
% ================================================================
figure;

subplot(2,2,1);
plot(t, F_meas_log(1,:), 'LineWidth', 1.2);
hold on;
plot(t, F_ref(1,:), '--', 'LineWidth', 1.2);
title('Contact Force Tracking');
xlabel('Time [s]');
ylabel('Force [N]');
legend('F_x measured', 'F_x ref');

subplot(2,2,2);
plot(t, x_log(1,:), 'LineWidth', 1.2);
hold on;
yline(x_wall, '--');
title('End-effector x Position');
xlabel('Time [s]');
ylabel('x [m]');
legend('x', 'wall');

subplot(2,2,3);
plot(t, tau_ctrl_log, 'LineWidth', 1.2);
title('Controller Torque');
xlabel('Time [s]');
ylabel('\tau [Nm]');
legend('\tau_1','\tau_2');

subplot(2,2,4);
plot(t, rad2deg(q), 'LineWidth', 1.2);
title('Joint Position');
xlabel('Time [s]');
ylabel('q [deg]');
legend('q_1','q_2');