%% demo_planar_2R_forward_dynamics
% 平面 2R 机械臂 forward dynamics 仿真
%
% Experiments
% -----------
% 1. Zero torque with gravity
% 2. Gravity compensation
% 3. Constant torque input
%
% Notes
% -----
% - 本脚本使用一个简化但标准的平面 2R 刚体动力学模型
% - 状态定义为 x = [q1; q2; dq1; dq2]
% - 动力学形式：
%       M(q) * ddq + C(q,dq) * dq + g(q) = tau
% - 数值积分使用 ode45
%
% Outputs
% -------
% - q(t), dq(t)
% - end-effector trajectory
%
% Parameters
% ----------
% l1, l2 : 连杆长度
% lc1, lc2 : 连杆质心到各关节的距离
% m1, m2 : 连杆质量
% I1, I2 : 连杆绕质心转动惯量
% g_const : 重力加速度
%
% Author Intent
% -------------
% - 先把"动力学存在"这件事跑出来
% - 暂时不做闭环控制，只看 forward dynamics 的自然响应

clear;
clc;
close all;

%% 1. 机器人参数
params.l1 = 1.0;
params.l2 = 0.8;

params.lc1 = 0.5;   % 假设质心在连杆中点
params.lc2 = 0.4;

params.m1 = 1.5;
params.m2 = 1.0;

% 细杆绕中心转动惯量近似：I = 1/12 * m * l^2
params.I1 = (1/12) * params.m1 * params.l1^2;
params.I2 = (1/12) * params.m2 * params.l2^2;

params.g_const = 9.81;

%% 2. 初始状态
% x = [q1; q2; dq1; dq2]
x0 = [
    pi/3;     % q1
   -pi/4;     % q2
    0.0;      % dq1
    0.0       % dq2
];

tspan = [0, 6];

%% 3. 三组实验
exp_list = {
    % struct('name', 'Zero Torque with Gravity', ...
    %        'tau_fun', @(t, x) [0; 0]), ...
    % struct('name', 'Gravity Compensation', ...
    %        'tau_fun', @(t, x) gravity_vector(x(1:2), params)), ...
    struct('name', 'Constant Torque Input', ...
           'tau_fun', @(t, x) [1.5; -0.8])
};

%% 4. 逐个实验仿真
for i_exp = 1:numel(exp_list)
    exp_cfg = exp_list{i_exp};

    ode_fun = @(t, x) robot_dynamics_2R(t, x, exp_cfg.tau_fun, params);

    [t, x] = ode45(ode_fun, tspan, x0);

    q  = x(:, 1:2);
    dq = x(:, 3:4);

    % 计算末端轨迹
    p = zeros(size(q,1), 2);
    for k = 1:size(q,1)
        p(k,:) = fkine_2R(q(k,:), params);
    end

    % 当前实验使用的力矩日志
    tau_log = zeros(size(q,1), 2);
    for k = 1:size(q,1)
        tau_log(k,:) = exp_cfg.tau_fun(t(k), x(k,:)')';
    end

    %% 4.1 joint position
    figure('Name', [exp_cfg.name, ' - Joint Position']);

    subplot(2,1,1);
    plot(t, q(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('q_1 (rad)');
    title([exp_cfg.name, ' - Joint 1 Position']);

    subplot(2,1,2);
    plot(t, q(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('q_2 (rad)');
    title([exp_cfg.name, ' - Joint 2 Position']);

    %% 4.2 joint velocity
    figure('Name', [exp_cfg.name, ' - Joint Velocity']);

    subplot(2,1,1);
    plot(t, dq(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('dq_1 (rad/s)');
    title([exp_cfg.name, ' - Joint 1 Velocity']);

    subplot(2,1,2);
    plot(t, dq(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('dq_2 (rad/s)');
    title([exp_cfg.name, ' - Joint 2 Velocity']);

    %% 4.3 torque
    figure('Name', [exp_cfg.name, ' - Torque']);

    subplot(2,1,1);
    plot(t, tau_log(:,1), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('\tau_1 (N·m)');
    title([exp_cfg.name, ' - Joint 1 Torque']);

    subplot(2,1,2);
    plot(t, tau_log(:,2), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('\tau_2 (N·m)');
    title([exp_cfg.name, ' - Joint 2 Torque']);

    %% 4.4 end-effector trajectory
    figure('Name', [exp_cfg.name, ' - End-effector Path']);
    plot(p(:,1), p(:,2), 'LineWidth', 2.0); hold on;
    plot(p(1,1), p(1,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8);
    plot(p(end,1), p(end,2), 's', 'MarkerSize', 8, 'LineWidth', 1.8);
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title([exp_cfg.name, ' - End-effector Trajectory']);
    legend('Path', 'Start', 'End', 'Location', 'best');

    %% 4.5 simple animation
    figure('Name', [exp_cfg.name, ' - Animation']);
    animate_planar_2R(q, params);
end

%% =========================
% 本地函数
% =========================

function dx = robot_dynamics_2R(t, x, tau_fun, params)
%ROBOT_DYNAMICS_2R 平面 2R 机械臂状态方程
%
% Parameters
% ----------
% t : double
%     时间
% x : (4,1) double
%     状态 [q1; q2; dq1; dq2]
% tau_fun : function_handle
%     输入力矩函数，格式 tau = tau_fun(t, x)
% params : struct
%     机械臂参数
%
% Returns
% -------
% dx : (4,1) double
%     状态导数 [dq1; dq2; ddq1; ddq2]

    q  = x(1:2);
    dq = x(3:4);

    tau = tau_fun(t, x);

    M = inertia_matrix(q, params);
    h = coriolis_centrifugal_vector(q, dq, params);
    g = gravity_vector(q, params);

    ddq = M \ (tau - h - g);

    dx = [
        dq;
        ddq
    ];
end

function M = inertia_matrix(q, params)
%INERTIA_MATRIX 计算平面 2R 机械臂惯性矩阵 M(q)
%
% Parameters
% ----------
% q : (2,1) double
%     关节角 [q1; q2]
% params : struct
%     参数结构体
%
% Returns
% -------
% M : (2,2) double
%     惯性矩阵

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

    M = [
        M11, M12;
        M21, M22
    ];
end

function h = coriolis_centrifugal_vector(q, dq, params)
%CORIOLIS_CENTRIFUGAL_VECTOR 计算速度相关项 h(q,dq) = C(q,dq)dq
%
% Parameters
% ----------
% q : (2,1) double
%     关节角
% dq : (2,1) double
%     关节角速度
% params : struct
%     参数结构体
%
% Returns
% -------
% h : (2,1) double
%     速度相关项

    q2  = q(2);
    dq1 = dq(1);
    dq2 = dq(2);

    l1  = params.l1;
    lc2 = params.lc2;
    m2  = params.m2;

    b = m2 * l1 * lc2 * sin(q2);

    h1 = -b * (2*dq1*dq2 + dq2^2);
    h2 =  b * dq1^2;

    h = [
        h1;
        h2
    ];
end

function g = gravity_vector(q, params)
%GRAVITY_VECTOR 计算重力项 g(q)
%
% Parameters
% ----------
% q : (2,1) double
%     关节角
% params : struct
%     参数结构体
%
% Returns
% -------
% g : (2,1) double
%     重力项

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

    g = [
        g1;
        g2
    ];
end

function p = fkine_2R(q, params)
%FKINE_2R 平面 2R 正运动学
%
% Parameters
% ----------
% q : (1,2) or (2,1) double
%     关节角 [q1, q2]
% params : struct
%     参数结构体
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

function animate_planar_2R(q_log, params)
%ANIMATE_PLANAR_2R 平面 2R 机械臂动画
%
% Parameters
% ----------
% q_log : (N,2) double
%     关节轨迹
% params : struct
%     参数结构体

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
        title('Planar 2R Forward Dynamics Animation');
        drawnow;
    end
end