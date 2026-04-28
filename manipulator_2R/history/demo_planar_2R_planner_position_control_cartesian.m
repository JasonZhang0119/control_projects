%% demo_planar_2R_planner_position_control
% Planner + Position Control + Dynamics 闭环仿真
%
% 功能
% ----
% 1. 使用 jtraj 生成 joint-space 参考轨迹
% 2. 基于平面 2R 动力学模型进行 forward dynamics 仿真
% 3. 对比三种位置控制器：
%    - PD
%    - PD + Gravity Compensation
%    - Computed Torque
% 4. 绘制 joint 跟踪、joint 误差、末端轨迹、控制力矩
%
% Notes
% -----
% - 机器人模型为平面 2R
% - 控制输入为关节力矩 tau
% - planner 输出 q_r, dq_r, ddq_r
% - 通过插值器在 ODE 内按连续时间读取参考轨迹
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

%% 2. Planner：生成参考轨迹
q_start = [0.2,  0.3];
q_goal  = [1.2, -0.9];

Tf = 4.0;
N  = 400;
t_ref = linspace(0, Tf, N)';

[q_ref, qd_ref, qdd_ref] = jtraj(q_start, q_goal, t_ref);

%% 3. 初始状态
x0 = [
    q_start(1);
    q_start(2);
    0.0;
    0.0
];

%% 4. 控制器参数
ctrl.Kp = diag([80, 60]);
ctrl.Kd = diag([18, 14]);

%% 5. 三种控制器配置
controllers = {
    % struct('name', 'PD', ...
    %        'mode', 'pd'), ...
    % struct('name', 'PD + Gravity Compensation', ...
    %        'mode', 'pd_g'), ...
    struct('name', 'Computed Torque', ...
           'mode', 'ctc')
};

%% 6. 逐个控制器仿真
for i_ctrl = 1:numel(controllers)
    cfg = controllers{i_ctrl};

    ode_fun = @(t, x) closed_loop_dynamics( ...
        t, x, t_ref, q_ref, qd_ref, qdd_ref, cfg.mode, ctrl, params);

    [t, x] = ode45(ode_fun, [0 Tf], x0);

    q  = x(:,1:2);
    dq = x(:,3:4);

    % 读取对应时刻的参考轨迹
    q_r_log   = interp1(t_ref, q_ref,   t, 'pchip');
    dq_r_log  = interp1(t_ref, qd_ref,  t, 'pchip');
    ddq_r_log = interp1(t_ref, qdd_ref, t, 'pchip');

    % 误差
    e_q  = q_r_log  - q;
    e_dq = dq_r_log - dq;

    % 记录控制输入
    tau_log = zeros(length(t), 2);
    for k = 1:length(t)
        tau_log(k,:) = controller_output( ...
            t(k), x(k,:)', t_ref, q_ref, qd_ref, qdd_ref, cfg.mode, ctrl, params)';
    end

    % 末端轨迹
    p_ref = zeros(length(t), 2);
    p_act = zeros(length(t), 2);

    for k = 1:length(t)
        p_ref(k,:) = fkine_2R(q_r_log(k,:), params);
        p_act(k,:) = fkine_2R(q(k,:), params);
    end

    %% 6.1 Joint Position Tracking
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

    %% 6.2 Joint Error
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

    %% 6.3 Joint Velocity Error
    figure('Name', [cfg.name, ' - Joint Velocity Error']);

    subplot(2,1,1);
    plot(t, e_dq(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_{dq1} (rad/s)');
    title([cfg.name, ' - Joint 1 Velocity Error']);

    subplot(2,1,2);
    plot(t, e_dq(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_{dq2} (rad/s)');
    title([cfg.name, ' - Joint 2 Velocity Error']);

    %% 6.4 End-effector Trajectory
    figure('Name', [cfg.name, ' - End-effector Trajectory']);
    plot(p_ref(:,1), p_ref(:,2), '--', 'LineWidth', 2.0); hold on;
    plot(p_act(:,1), p_act(:,2), 'LineWidth', 2.0);
    plot(p_ref(1,1), p_ref(1,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
    plot(p_ref(end,1), p_ref(end,2), 's', 'MarkerSize', 8, 'LineWidth', 1.8);
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title([cfg.name, ' - End-effector Trajectory']);
    legend('Reference', 'Actual', 'Start', 'Goal', 'Location', 'best');

    %% 6.5 Torque
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

    % %% 6.6 Animation
    % figure('Name', [cfg.name, ' - Animation']);
    % animate_planar_2R(q, params);
end

%% =========================
% 本地函数
% =========================

function dx = closed_loop_dynamics(t, x, t_ref, q_ref, qd_ref, qdd_ref, mode, ctrl, params)
%CLOSED_LOOP_DYNAMICS planner + controller + plant 闭环状态方程
%
% Parameters
% ----------
% t : double
%     当前时间
% x : (4,1) double
%     当前状态 [q1; q2; dq1; dq2]
% t_ref : (N,1) double
%     参考轨迹时间轴
% q_ref, qd_ref, qdd_ref : (N,2) double
%     参考位置、速度、加速度
% mode : char
%     控制模式：'pd', 'pd_g', 'ctc'
% ctrl : struct
%     控制器参数
% params : struct
%     动力学参数
%
% Returns
% -------
% dx : (4,1) double
%     状态导数

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
%CONTROLLER_OUTPUT 计算控制器输出力矩 tau
%
% Parameters
% ----------
% t : double
%     当前时间
% x : (4,1) double
%     当前状态
% t_ref : (N,1) double
%     参考时间轴
% q_ref, qd_ref, qdd_ref : (N,2) double
%     参考轨迹
% mode : char
%     控制模式
% ctrl : struct
%     Kp, Kd
% params : struct
%     动力学参数
%
% Returns
% -------
% tau : (2,1) double
%     控制力矩

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
%INERTIA_MATRIX 平面 2R 惯性矩阵 M(q)

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
%GRAVITY_VECTOR 重力项 g(q)

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