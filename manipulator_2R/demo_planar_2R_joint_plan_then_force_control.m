%% demo_planar_2R_joint_plan_then_force_control
% 平面 2R 机械臂：
% joint planning -> 先接近墙 -> 接触后法向力控制
%
% 功能
% ----
% 1. 使用 joint-space planner 生成接近阶段参考轨迹
% 2. 自由空间阶段使用位置控制（PD + gravity compensation）
% 3. 接触后切换为最小法向力控制
% 4. 环境采用竖直弹簧墙模型
% 5. 输出 joint / end-effector / contact force / torque 图
%
% 场景说明
% --------
% - 机械臂在平面内运动，末端与一面竖直墙接触
% - 墙位于 x = x_wall
% - 接触法向定义为 x 方向
% - 目标是在接触后维持期望法向力 F_d
%
% Notes
% -----
% - 这是"最小可运行"的力控脚本，不是工业级鲁棒实现
% - 接触阶段采用 Jacobian transpose force control
% - 为了避免关节在接触后完全漂移，叠加了轻微 posture stabilizer
% - 该脚本重点是把 planner + dynamics + contact + force control 串起来
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
env.x_wall = 1.20;       % 墙位置：x = x_wall
env.k_wall = 1500.0;     % 墙刚度 [N/m]
env.b_wall = 40.0;       % 墙阻尼 [N/(m/s)]

%% 3. 接近阶段 joint planner
q_start   = [0.15, 0.20];
q_approach = [0.95, -0.75];

Tf = 5.0;
N  = 500;
t_ref = linspace(0, Tf, N)';

[q_ref, qd_ref, qdd_ref] = jtraj(q_start, q_approach, t_ref);

%% 4. 控制参数
ctrl.Kp_pos = diag([90, 70]);
ctrl.Kd_pos = diag([18, 14]);

% 接触后 posture stabilizer（轻一点）
ctrl.Kp_post = diag([18, 12]);
ctrl.Kd_post = diag([5,  4]);

% 力控制增益
ctrl.Kf = 0.8;

% 期望法向接触力（正值表示压向墙）
ctrl.Fd = 8.0;   % [N]

% 接触切换阈值
ctrl.contact_force_threshold = 0.5;   % [N]

%% 5. 初始状态
x0 = [
    q_start(1);
    q_start(2);
    0.0;
    0.0
];

%% 6. 闭环仿真
ode_fun = @(t, x) closed_loop_force_dynamics( ...
    t, x, t_ref, q_ref, qd_ref, qdd_ref, ctrl, params, env);

[t, x] = ode45(ode_fun, [0 Tf], x0);

q  = x(:,1:2);
dq = x(:,3:4);

%% 7. 日志重建
q_r_log   = interp1(t_ref, q_ref,   t, 'pchip');
dq_r_log  = interp1(t_ref, qd_ref,  t, 'pchip');
ddq_r_log = interp1(t_ref, qdd_ref, t, 'pchip');

p_log      = zeros(length(t), 2);
pd_log     = zeros(length(t), 2);
F_env_log  = zeros(length(t), 2);
tau_log    = zeros(length(t), 2);
mode_log   = zeros(length(t), 1);   % 0: position, 1: force

for k = 1:length(t)
    qk  = q(k,:)';
    dqk = dq(k,:)';

    p_log(k,:)  = fkine_2R(qk, params);
    pd_log(k,:) = jacobian_2R(qk, params) * dqk;

    [F_env_k, ~] = environment_force(qk, dqk, params, env);
    F_env_log(k,:) = F_env_k';

    [tau_k, mode_k] = controller_output_force_mode( ...
        t(k), x(k,:)', t_ref, q_ref, qd_ref, qdd_ref, ctrl, params, env);

    tau_log(k,:) = tau_k';
    mode_log(k)  = mode_k;
end

%% 8. planner 参考末端轨迹
p_ref_log = zeros(length(t), 2);
for k = 1:length(t)
    p_ref_log(k,:) = fkine_2R(q_r_log(k,:), params);
end

%% 9. Joint Position Tracking
figure('Name', 'Joint Position Tracking');

subplot(2,1,1);
plot(t, q_r_log(:,1), '--', 'LineWidth', 1.8); hold on;
plot(t, q(:,1), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('Joint 1 Tracking');
legend('q_{1,r}', 'q_1', 'Location', 'best');

subplot(2,1,2);
plot(t, q_r_log(:,2), '--', 'LineWidth', 1.8); hold on;
plot(t, q(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('q_2 (rad)');
title('Joint 2 Tracking');
legend('q_{2,r}', 'q_2', 'Location', 'best');

%% 10. Joint Velocity
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

%% 11. End-effector Path
figure('Name', 'End-effector Path');
plot(p_ref_log(:,1), p_ref_log(:,2), '--', 'LineWidth', 2.0); hold on;
plot(p_log(:,1), p_log(:,2), 'LineWidth', 2.0);
xline(env.x_wall, 'k--', 'LineWidth', 1.5);
plot(p_log(1,1), p_log(1,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
plot(p_log(end,1), p_log(end,2), 's', 'MarkerSize', 8, 'LineWidth', 1.8);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('End-effector Path');
legend('Reference path', 'Actual path', 'Wall', 'Start', 'End', 'Location', 'best');

%% 12. Cartesian Position vs Time
figure('Name', 'Cartesian Position vs Time');

subplot(2,1,1);
plot(t, p_ref_log(:,1), '--', 'LineWidth', 1.8); hold on;
plot(t, p_log(:,1), 'LineWidth', 1.8);
yline(env.x_wall, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x');
title('End-effector X Position');
legend('x_r', 'x', 'x_{wall}', 'Location', 'best');

subplot(2,1,2);
plot(t, p_ref_log(:,2), '--', 'LineWidth', 1.8); hold on;
plot(t, p_log(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('y');
title('End-effector Y Position');
legend('y_r', 'y', 'Location', 'best');

%% 13. Contact Force
figure('Name', 'Contact Force');

subplot(2,1,1);
plot(t, F_env_log(:,1), 'LineWidth', 1.8); hold on;
yline(ctrl.Fd, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('F_x (N)');
title('Normal Contact Force');
legend('F_{env,x}', 'F_d', 'Location', 'best');

subplot(2,1,2);
plot(t, F_env_log(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('F_y (N)');
title('Tangential Contact Force');

%% 14. Control Torque
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

%% 15. Controller Mode
figure('Name', 'Controller Mode');
stairs(t, mode_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Mode');
yticks([0 1]);
yticklabels({'Position', 'Force'});
title('Controller Switching Mode');

%% 16. Animation
figure('Name', 'Animation');
animate_planar_2R_with_wall(q, params, env);

%% =========================
% 本地函数
% =========================

function dx = closed_loop_force_dynamics(t, x, t_ref, q_ref, qd_ref, qdd_ref, ctrl, params, env)
%CLOSED_LOOP_FORCE_DYNAMICS planner + controller + contact dynamics
%
% Parameters
% ----------
% t : double
%     当前时间
% x : (4,1) double
%     当前状态 [q1; q2; dq1; dq2]
% t_ref : (N,1) double
%     planner 时间轴
% q_ref, qd_ref, qdd_ref : (N,2) double
%     planner 输出的关节参考轨迹
% ctrl : struct
%     控制器参数
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

    tau = controller_output_force_mode( ...
        t, x, t_ref, q_ref, qd_ref, qdd_ref, ctrl, params, env);

    M = inertia_matrix(q, params);
    h = coriolis_centrifugal_vector(q, dq, params);
    g = gravity_vector(q, params);

    [F_env, J] = environment_force(q, dq, params, env);

    % 环境力对关节的反作用力矩
    tau_env = J' * F_env;

    ddq = M \ (tau + tau_env - h - g);

    dx = [dq; ddq];
end

function [tau, mode_flag] = controller_output_force_mode(t, x, t_ref, q_ref, qd_ref, qdd_ref, ctrl, params, env)
%CONTROLLER_OUTPUT_FORCE_MODE 两阶段控制器：
% 1. 自由空间：joint position control
% 2. 接触后：normal force control + posture stabilizer
%
% Parameters
% ----------
% t : double
%     当前时间
% x : (4,1) double
%     当前状态
% t_ref : (N,1) double
%     参考时间
% q_ref, qd_ref, qdd_ref : (N,2) double
%     参考关节轨迹
% ctrl : struct
%     控制参数
% params : struct
%     机器人参数
% env : struct
%     环境参数
%
% Returns
% -------
% tau : (2,1) double
%     控制力矩
% mode_flag : double
%     0 表示位置控制阶段，1 表示力控制阶段

    q  = x(1:2);
    dq = x(3:4);

    qr   = interp1(t_ref, q_ref,   t, 'pchip')';
    dqr  = interp1(t_ref, qd_ref,  t, 'pchip')';
    ddqr = interp1(t_ref, qdd_ref, t, 'pchip')'; %#ok<NASGU>

    e  = qr  - q;
    de = dqr - dq;

    [F_env, J] = environment_force(q, dq, params, env);
    F_contact = max(0, -F_env(1));   % 法向接触力，取正值表示压墙力

    if F_contact < ctrl.contact_force_threshold
        % -------- 阶段 1：位置控制（接近阶段） --------
        tau = ctrl.Kp_pos * e + ctrl.Kd_pos * de + gravity_vector(q, params);
        mode_flag = 0;
    else
        % -------- 阶段 2：接触后法向力控制 --------
        % 目标：维持 F_contact -> Fd
        force_error = ctrl.Fd - F_contact;
        Fx_cmd = ctrl.Fd + ctrl.Kf * force_error;

        F_cmd_task = [Fx_cmd; 0.0];
        tau_force = J' * F_cmd_task;

        % 叠加轻微 posture stabilizer，避免漂移过大
        tau_post = ctrl.Kp_post * e + ctrl.Kd_post * de;

        tau = tau_force + tau_post + gravity_vector(q, params);
        mode_flag = 1;
    end
end

function [F_env, J] = environment_force(q, dq, params, env)
%ENVIRONMENT_FORCE 竖直弹簧墙接触力模型
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
%     环境参数
%
% Returns
% -------
% F_env : (2,1) double
%     作用在末端上的环境力
% J : (2,2) double
%     末端雅可比矩阵
%
% Notes
% -----
% - 墙位于 x = x_wall
% - 当 x_ee > x_wall 时，认为穿入墙体，产生向左的反力（负 x）
% - 阻尼只在接触时生效

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
%CORIOLIS_CENTRIFUGAL_VECTOR 速度相关项 h(q,dq)=C(q,dq)dq

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
%
% Parameters
% ----------
% q : (1,2) or (2,1) double
%     关节角
% params : struct
%     机器人参数
%
% Returns
% -------
% p : (1,2) double
%     末端位置 [x, y]

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
%JACOBIAN_2R 平面 2R 末端雅可比矩阵
%
% Parameters
% ----------
% q : (1,2) or (2,1) double
%     关节角
% params : struct
%     机器人参数
%
% Returns
% -------
% J : (2,2) double
%     平面位置雅可比矩阵

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

function animate_planar_2R_with_wall(q_log, params, env)
%ANIMATE_PLANAR_2R_WITH_WALL 平面 2R + 墙动画
%
% Parameters
% ----------
% q_log : (N,2) double
%     关节轨迹
% params : struct
%     机器人参数
% env : struct
%     环境参数

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
        hold off;

        grid on;
        axis equal;
        axis([-0.5, 2.0, -1.5, 1.8]);
        xlabel('X');
        ylabel('Y');
        title('Planar 2R Force Control Animation');
        drawnow;
    end
end