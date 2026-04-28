%% demo_planar_2R_impedance_control
% 平面 2R 机械臂：Cartesian Impedance Control + 墙接触
%
% 功能
% ----
% 1. 末端目标位置给定
% 2. 使用 Cartesian impedance 生成期望末端作用力
% 3. 使用 Jacobian transpose 将末端力映射为关节力矩
% 4. 在竖直弹簧墙环境中观察柔顺接触行为
%
% 控制结构
% --------
% F_cmd = Kx * (x_r - x) + Dx * (dx_r - dx)
% tau   = J(q)' * F_cmd + g(q)
%
% Notes
% -----
% - 这是最小阻抗控制脚本，重点是行为直观
% - 当前未加入完整 inverse dynamics，只加重力补偿
% - 若后续要升级，可进一步加：
%   1. operational space inertia shaping
%   2. full computed torque inner loop
%   3. hybrid position/force control
%
% 状态定义
% --------
% x = [q1; q2; dq1; dq2]

clear;
clc;
close all;

%% 1. 机器人参数
params.l1 = 1.0;
params.l2 = 0.8;

params.lc1 = 0.5;
params.lc2 = 0.4;

params.m1 = 1.5;
params.m2 = 1.0;

params.I1 = (1/12) * params.m1 * params.l1^2;
params.I2 = (1/12) * params.m2 * params.l2^2;

params.g_const = 9.81;

%% 2. 环境参数（竖直弹簧墙）
env.x_wall = 1.15;
env.k_wall = 1800.0;
env.b_wall = 50.0;

%% 3. 初始关节姿态
q0 = [0.25; 0.35];
dq0 = [0.0; 0.0];
x0 = [q0; dq0];

%% 4. 末端目标位置
% 目标点故意放在墙后一点，这样会发生接触
ctrl.x_ref = [1.28; 0.55];
ctrl.dx_ref = [0.0; 0.0];

%% 5. 阻抗参数
% F_cmd = Kx * (x_r - x) + Dx * (dx_r - dx)
ctrl.Kx = diag([350, 250]);
ctrl.Dx = diag([45,  30]);

%% 6. 仿真时间
tspan = [0, 6];

%% 7. 闭环仿真
ode_fun = @(t, x) impedance_closed_loop_dynamics(t, x, ctrl, params, env);
[t, x] = ode45(ode_fun, tspan, x0);

q  = x(:,1:2);
dq = x(:,3:4);

%% 8. 日志重建
p_log      = zeros(length(t), 2);
pd_log     = zeros(length(t), 2);
F_cmd_log  = zeros(length(t), 2);
F_env_log  = zeros(length(t), 2);
tau_log    = zeros(length(t), 2);
penetration_log = zeros(length(t), 1);

for k = 1:length(t)
    qk  = q(k,:)';
    dqk = dq(k,:)';

    p_log(k,:)  = fkine_2R(qk, params);
    pd_log(k,:) = jacobian_2R(qk, params) * dqk;

    [F_env_k, ~, pen_k] = environment_force(qk, dqk, params, env);
    F_env_log(k,:) = F_env_k';
    penetration_log(k) = pen_k;

    [tau_k, F_cmd_k] = impedance_controller(qk, dqk, ctrl, params);
    tau_log(k,:)   = tau_k';
    F_cmd_log(k,:) = F_cmd_k';
end

%% 9. 目标位置线
x_ref_log = ctrl.x_ref(1) * ones(size(t));
y_ref_log = ctrl.x_ref(2) * ones(size(t));

%% 10. 末端路径图
figure('Name', 'End-effector Path');
plot(p_log(:,1), p_log(:,2), 'LineWidth', 2.0); hold on;
plot(p_log(1,1), p_log(1,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
plot(p_log(end,1), p_log(end,2), 's', 'MarkerSize', 8, 'LineWidth', 1.8);
plot(ctrl.x_ref(1), ctrl.x_ref(2), 'x', 'MarkerSize', 10, 'LineWidth', 2.0);
xline(env.x_wall, 'k--', 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('End-effector Path under Impedance Control');
legend('Actual path', 'Start', 'End', 'Reference point', 'Wall', 'Location', 'best');

%% 11. Cartesian Position vs Time
figure('Name', 'Cartesian Position vs Time');

subplot(2,1,1);
plot(t, p_log(:,1), 'LineWidth', 1.8); hold on;
plot(t, x_ref_log, '--', 'LineWidth', 1.5);
yline(env.x_wall, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x');
title('End-effector X Position');
legend('x', 'x_r', 'x_{wall}', 'Location', 'best');

subplot(2,1,2);
plot(t, p_log(:,2), 'LineWidth', 1.8); hold on;
plot(t, y_ref_log, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('y');
title('End-effector Y Position');
legend('y', 'y_r', 'Location', 'best');

%% 12. Cartesian Velocity vs Time
figure('Name', 'Cartesian Velocity vs Time');

subplot(2,1,1);
plot(t, pd_log(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('dx');
title('End-effector X Velocity');

subplot(2,1,2);
plot(t, pd_log(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('dy');
title('End-effector Y Velocity');

%% 13. Commanded Force and Environment Force
figure('Name', 'Commanded Force and Environment Force');

subplot(2,1,1);
plot(t, F_cmd_log(:,1), 'LineWidth', 1.8); hold on;
plot(t, F_env_log(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Force X (N)');
title('Normal-direction Force');
legend('F_{cmd,x}', 'F_{env,x}', 'Location', 'best');

subplot(2,1,2);
plot(t, F_cmd_log(:,2), 'LineWidth', 1.8); hold on;
plot(t, F_env_log(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Force Y (N)');
title('Tangential-direction Force');
legend('F_{cmd,y}', 'F_{env,y}', 'Location', 'best');

%% 14. Joint Position
figure('Name', 'Joint Position');

subplot(2,1,1);
plot(t, q(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('Joint 1 Position');

subplot(2,1,2);
plot(t, q(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('q_2 (rad)');
title('Joint 2 Position');

%% 15. Joint Velocity
figure('Name', 'Joint Velocity');

subplot(2,1,1);
plot(t, dq(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('dq_1 (rad/s)');
title('Joint 1 Velocity');

subplot(2,1,2);
plot(t, dq(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('dq_2 (rad/s)');
title('Joint 2 Velocity');

%% 16. Control Torque
figure('Name', 'Control Torque');

subplot(2,1,1);
plot(t, tau_log(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('\tau_1 (N·m)');
title('Joint 1 Torque');

subplot(2,1,2);
plot(t, tau_log(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('\tau_2 (N·m)');
title('Joint 2 Torque');

%% 17. Penetration
figure('Name', 'Wall Penetration');
plot(t, penetration_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('penetration (m)');
title('Wall Penetration');

%% 18. Animation
figure('Name', 'Animation');
animate_planar_2R_with_wall_and_target(q, params, env, ctrl);

%% =========================
% 本地函数
% =========================

function dx = impedance_closed_loop_dynamics(~, x, ctrl, params, env)
%IMPEDANCE_CLOSED_LOOP_DYNAMICS 阻抗控制闭环动力学
%
% Parameters
% ----------
% x : (4,1) double
%     状态 [q1; q2; dq1; dq2]
% ctrl : struct
%     阻抗控制参数
% params : struct
%     机器人动力学参数
% env : struct
%     环境参数
%
% Returns
% -------
% dx : (4,1) double
%     状态导数

    q  = x(1:2);
    dq = x(3:4);

    [tau_ctrl, ~] = impedance_controller(q, dq, ctrl, params);

    M = inertia_matrix(q, params);
    h = coriolis_centrifugal_vector(q, dq, params);
    g = gravity_vector(q, params);

    [F_env, J, ~] = environment_force(q, dq, params, env);
    tau_env = J' * F_env;

    ddq = M \ (tau_ctrl + tau_env - h - g);

    dx = [dq; ddq];
end

function [tau, F_cmd] = impedance_controller(q, dq, ctrl, params)
%IMPEDANCE_CONTROLLER Cartesian 阻抗控制器
%
% Parameters
% ----------
% q : (2,1) double
%     当前关节角
% dq : (2,1) double
%     当前关节角速度
% ctrl : struct
%     控制参数，包含 x_ref, dx_ref, Kx, Dx
% params : struct
%     机器人参数
%
% Returns
% -------
% tau : (2,1) double
%     控制力矩
% F_cmd : (2,1) double
%     期望末端作用力

    p  = fkine_2R(q, params)';
    J  = jacobian_2R(q, params);
    pd = J * dq;

    e_p  = ctrl.x_ref  - p;
    e_pd = ctrl.dx_ref - pd;

    F_cmd = ctrl.Kx * e_p + ctrl.Dx * e_pd;

    tau = J' * F_cmd + gravity_vector(q, params);
end

function [F_env, J, penetration] = environment_force(q, dq, params, env)
%ENVIRONMENT_FORCE 竖直弹簧墙环境力
%
% Parameters
% ----------
% q : (2,1) double
%     当前关节角
% dq : (2,1) double
%     当前关节角速度
% params : struct
%     机器人参数
% env : struct
%     墙参数
%
% Returns
% -------
% F_env : (2,1) double
%     作用在末端的环境力
% J : (2,2) double
%     雅可比矩阵
% penetration : double
%     穿透深度

    p  = fkine_2R(q, params);
    J  = jacobian_2R(q, params);
    pd = J * dq;

    x = p(1);
    xd = pd(1);

    penetration = x - env.x_wall;

    if penetration > 0
        Fx = -env.k_wall * penetration - env.b_wall * max(0, xd);
    else
        Fx = 0.0;
        penetration = 0.0;
    end

    F_env = [Fx; 0.0];
end

function M = inertia_matrix(q, params)
%INERTIA_MATRIX 平面 2R 惯性矩阵

    q2 = q(2);

    l1  = params.l1;
    lc1 = params.lc1;
    lc2 = params.lc2;
    m1  = params.m1;
    m2  = params.m2;
    I1  = params.I1;
    I2  = params.I2;

    M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2));
    M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q2));
    M21 = M12;
    M22 = I2 + m2*lc2^2;

    M = [M11, M12;
         M21, M22];
end

function h = coriolis_centrifugal_vector(q, dq, params)
%CORIOLIS_CENTRIFUGAL_VECTOR 速度相关项

    q2  = q(2);
    dq1 = dq(1);
    dq2 = dq(2);

    l1  = params.l1;
    lc2 = params.lc2;
    m2  = params.m2;

    b = m2 * l1 * lc2 * sin(q2);

    h1 = -b * (2*dq1*dq2 + dq2^2);
    h2 =  b * dq1^2;

    h = [h1; h2];
end

function g = gravity_vector(q, params)
%GRAVITY_VECTOR 重力项

    q1 = q(1);
    q2 = q(2);

    l1  = params.l1;
    lc1 = params.lc1;
    lc2 = params.lc2;
    m1  = params.m1;
    m2  = params.m2;
    g0  = params.g_const;

    g1 = (m1*lc1 + m2*l1) * g0 * cos(q1) + m2*lc2*g0*cos(q1 + q2);
    g2 = m2 * lc2 * g0 * cos(q1 + q2);

    g = [g1; g2];
end

function p = fkine_2R(q, params)
%FKINE_2R 平面 2R 正运动学

    q = q(:);
    q1 = q(1);
    q2 = q(2);

    l1 = params.l1;
    l2 = params.l2;

    x = l1*cos(q1) + l2*cos(q1 + q2);
    y = l1*sin(q1) + l2*sin(q1 + q2);

    p = [x, y];
end

function J = jacobian_2R(q, params)
%JACOBIAN_2R 平面 2R 位置雅可比矩阵

    q = q(:);
    q1 = q(1);
    q2 = q(2);

    l1 = params.l1;
    l2 = params.l2;

    J = [
        -l1*sin(q1) - l2*sin(q1 + q2),  -l2*sin(q1 + q2);
         l1*cos(q1) + l2*cos(q1 + q2),   l2*cos(q1 + q2)
    ];
end

function animate_planar_2R_with_wall_and_target(q_log, params, env, ctrl)
%ANIMATE_PLANAR_2R_WITH_WALL_AND_TARGET 平面 2R + 墙 + 目标点动画
%
% Parameters
% ----------
% q_log : (N,2) double
%     关节轨迹
% params : struct
%     机器人参数
% env : struct
%     墙参数
% ctrl : struct
%     目标点信息

    l1 = params.l1;
    l2 = params.l2;

    for k = 1:size(q_log,1)
        q1 = q_log(k,1);
        q2 = q_log(k,2);

        p0 = [0, 0];
        p1 = [l1*cos(q1), l1*sin(q1)];
        p2 = [p1(1) + l2*cos(q1+q2), p1(2) + l2*sin(q1+q2)];

        plot([p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], '-o', ...
            'LineWidth', 2, 'MarkerSize', 6);
        hold on;
        xline(env.x_wall, 'k--', 'LineWidth', 1.5);
        plot(ctrl.x_ref(1), ctrl.x_ref(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2.0);
        hold off;

        grid on;
        axis equal;
        axis([-0.5, 2.0, -1.2, 1.8]);
        xlabel('X');
        ylabel('Y');
        title('Planar 2R Impedance Control Animation');
        drawnow;
    end
end