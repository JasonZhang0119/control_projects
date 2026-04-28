%% demo_planar_2R_cartesian_plan_then_IK_control
% 末端位姿给定 -> 先规划末端轨迹 -> 再逐点 IK -> 位置控制闭环
%
% 功能
% ----
% 1. 给定末端起点与终点（Cartesian position）
% 2. 在 task space 中生成平滑参考轨迹 p_r(t), v_r(t), a_r(t)
% 3. 对每个时刻逐点做 IK，得到 q_r(t)
% 4. 对 q_r(t) 数值微分，得到 dq_r(t), ddq_r(t)
% 5. 基于 2R 动力学模型，分别比较：
%    - PD
%    - PD + Gravity Compensation
%    - Computed Torque
%
% Notes
% -----
% - 这里只规划平面 2R 的末端位置 [x, y]
% - IK 采用解析逆解，并通过"邻近上一时刻解"保持连续性
% - dq_r, ddq_r 来自数值微分，因此边界处会略粗糙
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

%% 2. 给定末端起点与终点
p_start = [1.45, 0.20];
p_goal  = [0.75, 1.00];

% 可达性检查
check_reachable_2R(p_start, params);
check_reachable_2R(p_goal, params);

%% 3. task-space planner：生成 p_r(t), v_r(t), a_r(t)
Tf = 4.0;
N  = 400;
t_ref = linspace(0, Tf, N)';

% 用 quintic 时间标度 s(t) 从 0 -> 1
[s, sd, sdd] = quintic_time_scaling(t_ref, Tf);

delta_p = p_goal - p_start;

p_ref   = zeros(N, 2);
pd_ref  = zeros(N, 2);
pdd_ref = zeros(N, 2);

for k = 1:N
    p_ref(k,:)   = p_start + s(k)   * delta_p;
    pd_ref(k,:)  =               sd(k)  * delta_p;
    pdd_ref(k,:) =               sdd(k) * delta_p;
end

%% 4. 对 task-space 轨迹逐点做 IK，得到 q_r(t)
q_ref = zeros(N, 2);

q_ref(1,:) = ik_2R_continuous(p_ref(1,:), params, []);

for k = 2:N
    q_ref(k,:) = ik_2R_continuous(p_ref(k,:), params, q_ref(k-1,:));
end

%% 5. 对 q_r 数值微分，得到 dq_r, ddq_r
dt = t_ref(2) - t_ref(1);

qd_ref  = numerical_diff(q_ref, dt);
qdd_ref = numerical_diff(qd_ref, dt);

%% 6. 用 FK 验证 task-space 规划经过 IK 后是否仍贴近原始轨迹
p_ref_check = zeros(N, 2);
for k = 1:N
    p_ref_check(k,:) = fkine_2R(q_ref(k,:), params);
end

%% 7. 初始状态
x0 = [
    q_ref(1,1);
    q_ref(1,2);
    0.0;
    0.0
];

%% 8. 控制器参数
ctrl.Kp = diag([80, 60]);
ctrl.Kd = diag([18, 14]);

%% 9. 三种控制器配置
controllers = {
    % struct('name', 'PD', ...
    %        'mode', 'pd'), ...
    % struct('name', 'PD + Gravity Compensation', ...
    %        'mode', 'pd_g'), ...
    struct('name', 'Computed Torque', ...
           'mode', 'ctc')
};

%% 10. 先画 planner / IK 结果
figure('Name', 'Task-space Planned Path vs FK-verified Path');
plot(p_ref(:,1), p_ref(:,2), '--', 'LineWidth', 2.0); hold on;
plot(p_ref_check(:,1), p_ref_check(:,2), 'LineWidth', 1.8);
plot(p_start(1), p_start(2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
plot(p_goal(1),  p_goal(2),  's', 'MarkerSize', 8, 'LineWidth', 1.8);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Cartesian Planned Path vs FK-verified Path');
legend('Planned path', 'FK-verified path', 'Start', 'Goal', 'Location', 'best');

figure('Name', 'Reference Joint Trajectories from Cartesian Planning');

subplot(2,1,1);
plot(t_ref, q_ref(:,1), 'LineWidth', 1.8); hold on;
plot(t_ref, q_ref(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('q (rad)');
title('q_r(t)');
legend('q_{1,r}', 'q_{2,r}', 'Location', 'best');

subplot(2,1,2);
plot(t_ref, qd_ref(:,1), 'LineWidth', 1.8); hold on;
plot(t_ref, qd_ref(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('dq_r (rad/s)');
title('dq_r(t)');
legend('dq_{1,r}', 'dq_{2,r}', 'Location', 'best');

figure('Name', 'Reference Cartesian Signals');

subplot(3,1,1);
plot(t_ref, p_ref(:,1), 'LineWidth', 1.8); hold on;
plot(t_ref, p_ref(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('p_r');
title('Cartesian Position Reference');
legend('x_r', 'y_r', 'Location', 'best');

subplot(3,1,2);
plot(t_ref, pd_ref(:,1), 'LineWidth', 1.8); hold on;
plot(t_ref, pd_ref(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('v_r');
title('Cartesian Velocity Reference');
legend('dx_r', 'dy_r', 'Location', 'best');

subplot(3,1,3);
plot(t_ref, pdd_ref(:,1), 'LineWidth', 1.8); hold on;
plot(t_ref, pdd_ref(:,2), 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('a_r');
title('Cartesian Acceleration Reference');
legend('ddx_r', 'ddy_r', 'Location', 'best');

%% 11. 逐个控制器仿真
for i_ctrl = 1:numel(controllers)
    cfg = controllers{i_ctrl};

    ode_fun = @(t, x) closed_loop_dynamics( ...
        t, x, t_ref, q_ref, qd_ref, qdd_ref, cfg.mode, ctrl, params);

    [t, x] = ode45(ode_fun, [0 Tf], x0);

    q  = x(:,1:2);
    dq = x(:,3:4);

    % 插值对应参考轨迹
    q_r_log   = interp1(t_ref, q_ref,   t, 'pchip');
    dq_r_log  = interp1(t_ref, qd_ref,  t, 'pchip');
    ddq_r_log = interp1(t_ref, qdd_ref, t, 'pchip');

    % 误差
    e_q  = q_r_log  - q;
    e_dq = dq_r_log - dq;

    % 力矩
    tau_log = zeros(length(t), 2);
    for k = 1:length(t)
        tau_log(k,:) = controller_output( ...
            t(k), x(k,:)', t_ref, q_ref, qd_ref, qdd_ref, cfg.mode, ctrl, params)';
    end

    % task-space 实际/参考
    p_r_log = zeros(length(t), 2);
    p_act   = zeros(length(t), 2);

    for k = 1:length(t)
        p_r_log(k,:) = interp1(t_ref, p_ref, t(k), 'pchip');
        p_act(k,:)   = fkine_2R(q(k,:), params);
    end

    %% 11.1 Joint Position Tracking
    figure('Name', [cfg.name, ' - Joint Position Tracking']);

    subplot(2,1,1);
    plot(t, q_r_log(:,1), '--', 'LineWidth', 1.8); hold on;
    plot(t, q(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('q_1 (rad)');
    title([cfg.name, ' - Joint 1 Tracking']);
    legend('q_{1,r}', 'q_1', 'Location', 'best');

    subplot(2,1,2);
    plot(t, q_r_log(:,2), '--', 'LineWidth', 1.8); hold on;
    plot(t, q(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('q_2 (rad)');
    title([cfg.name, ' - Joint 2 Tracking']);
    legend('q_{2,r}', 'q_2', 'Location', 'best');

    %% 11.2 Joint Error
    figure('Name', [cfg.name, ' - Joint Error']);

    subplot(2,1,1);
    plot(t, e_q(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_{q1} (rad)');
    title([cfg.name, ' - Joint 1 Error']);

    subplot(2,1,2);
    plot(t, e_q(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_{q2} (rad)');
    title([cfg.name, ' - Joint 2 Error']);

    %% 11.3 Task-space Trajectory
    figure('Name', [cfg.name, ' - End-effector Trajectory']);
    plot(p_r_log(:,1), p_r_log(:,2), '--', 'LineWidth', 2.0); hold on;
    plot(p_act(:,1), p_act(:,2), 'LineWidth', 2.0);
    plot(p_r_log(1,1), p_r_log(1,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
    plot(p_r_log(end,1), p_r_log(end,2), 's', 'MarkerSize', 8, 'LineWidth', 1.8);
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title([cfg.name, ' - End-effector Trajectory']);
    legend('Reference', 'Actual', 'Start', 'Goal', 'Location', 'best');

    %% 11.4 Task-space Error
    e_p = p_r_log - p_act;

    figure('Name', [cfg.name, ' - Task-space Error']);

    subplot(2,1,1);
    plot(t, e_p(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_x');
    title([cfg.name, ' - X Error']);

    subplot(2,1,2);
    plot(t, e_p(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_y');
    title([cfg.name, ' - Y Error']);

    %% 11.5 Torque
    figure('Name', [cfg.name, ' - Control Torque']);

    subplot(2,1,1);
    plot(t, tau_log(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('\tau_1 (N·m)');
    title([cfg.name, ' - Joint 1 Torque']);

    subplot(2,1,2);
    plot(t, tau_log(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('\tau_2 (N·m)');
    title([cfg.name, ' - Joint 2 Torque']);

    %% 11.6 Animation
    % figure('Name', [cfg.name, ' - Animation']);
    % animate_planar_2R(q, params);
end

%% =========================
% 本地函数
% =========================

function dx = closed_loop_dynamics(t, x, t_ref, q_ref, qd_ref, qdd_ref, mode, ctrl, params)
%CLOSED_LOOP_DYNAMICS 闭环动力学

    q  = x(1:2);
    dq = x(3:4);

    tau = controller_output(t, x, t_ref, q_ref, qd_ref, qdd_ref, mode, ctrl, params);

    M = inertia_matrix(q, params);
    h = coriolis_centrifugal_vector(q, dq, params);
    g = gravity_vector(q, params);

    ddq = M \ (tau - h - g);

    dx = [dq; ddq];
end

function tau = controller_output(t, x, t_ref, q_ref, qd_ref, qdd_ref, mode, ctrl, params)
%CONTROLLER_OUTPUT 计算控制力矩

    q  = x(1:2);
    dq = x(3:4);

    qr   = interp1(t_ref, q_ref,   t, 'pchip')';
    dqr  = interp1(t_ref, qd_ref,  t, 'pchip')';
    ddqr = interp1(t_ref, qdd_ref, t, 'pchip')';

    e  = qr  - q;
    de = dqr - dq;

    Kp = ctrl.Kp;
    Kd = ctrl.Kd;

    switch mode
        case 'pd'
            tau = Kp * e + Kd * de;

        case 'pd_g'
            tau = Kp * e + Kd * de + gravity_vector(q, params);

        case 'ctc'
            v = ddqr + Kd * de + Kp * e;
            M = inertia_matrix(q, params);
            h = coriolis_centrifugal_vector(q, dq, params);
            g = gravity_vector(q, params);

            tau = M * v + h + g;

        otherwise
            error('未知控制模式: %s', mode);
    end
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

function q = ik_2R_continuous(p, params, q_prev)
%IK_2R_CONTINUOUS 解析 IK + 连续分支选择
%
% Parameters
% ----------
% p : (1,2) double
%     末端位置 [x, y]
% params : struct
%     机器人参数
% q_prev : (1,2) double or []
%     上一时刻关节解
%
% Returns
% -------
% q : (1,2) double
%     当前选择的连续关节解

    x = p(1);
    y = p(2);

    l1 = params.l1;
    l2 = params.l2;

    c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
    c2 = min(max(c2, -1), 1);

    s2_pos = sqrt(1 - c2^2);
    s2_neg = -sqrt(1 - c2^2);

    q2_a = atan2(s2_pos, c2);
    q2_b = atan2(s2_neg, c2);

    q1_a = atan2(y, x) - atan2(l2*sin(q2_a), l1 + l2*cos(q2_a));
    q1_b = atan2(y, x) - atan2(l2*sin(q2_b), l1 + l2*cos(q2_b));

    qa = [wrap_to_pi_local(q1_a), wrap_to_pi_local(q2_a)];
    qb = [wrap_to_pi_local(q1_b), wrap_to_pi_local(q2_b)];

    if isempty(q_prev)
        q = qa;
    else
        da = norm(angle_diff_vec(qa, q_prev));
        db = norm(angle_diff_vec(qb, q_prev));

        if da <= db
            q = qa;
        else
            q = qb;
        end
    end
end

function d = angle_diff_vec(a, b)
%ANGLE_DIFF_VEC 角差向量
    d = [wrap_to_pi_local(a(1)-b(1)), wrap_to_pi_local(a(2)-b(2))];
end

function a = wrap_to_pi_local(a)
%WRAP_TO_PI_LOCAL 包裹到 [-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end

function check_reachable_2R(p, params)
%CHECK_REACHABLE_2R 检查点是否可达

    r = hypot(p(1), p(2));
    l1 = params.l1;
    l2 = params.l2;

    if r > (l1 + l2) || r < abs(l1 - l2)
        error('点 [%.3f, %.3f] 不可达。', p(1), p(2));
    end
end

function [s, sd, sdd] = quintic_time_scaling(t, Tf)
%QUINTIC_TIME_SCALING 五次时间标度
%
% Parameters
% ----------
% t : (N,1) double
%     时间向量
% Tf : double
%     总时间
%
% Returns
% -------
% s : (N,1) double
%     位置标度
% sd : (N,1) double
%     速度标度
% sdd : (N,1) double
%     加速度标度

    tau = t / Tf;

    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    sd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / Tf;
    sdd = (60*tau - 180*tau.^2 + 120*tau.^3) / Tf^2;
end

function dq = numerical_diff(q, dt)
%NUMERICAL_DIFF 数值微分
%
% Parameters
% ----------
% q : (N,m) double
%     离散轨迹
% dt : double
%     采样周期
%
% Returns
% -------
% dq : (N,m) double
%     数值导数

    dq = zeros(size(q));

    if size(q,1) < 2
        return;
    end

    dq(1,:) = (q(2,:) - q(1,:)) / dt;

    for k = 2:size(q,1)-1
        dq(k,:) = (q(k+1,:) - q(k-1,:)) / (2*dt);
    end

    dq(end,:) = (q(end,:) - q(end-1,:)) / dt;
end

function animate_planar_2R(q_log, params)
%ANIMATE_PLANAR_2R 平面 2R 动画

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
        grid on;
        axis equal;
        axis([-2, 2, -2, 2]);
        xlabel('X');
        ylabel('Y');
        title('Planar 2R Closed-loop Tracking Animation');
        drawnow;
    end
end