%% demo_planar_2R_compare_4routes
% 比较四种规划路线：
% Route 1: Cartesian waypoints -> IK -> joint-space jtraj
% Route 2: Cartesian linear interpolation -> pointwise IK
% Route 3: Cartesian waypoints -> IK -> joint-space LSPB
% Route 4: Cartesian LSPB interpolation -> pointwise IK
%
% 同时绘制：
% 1. 笛卡尔路径对比图
% 2. Joint 位置图
% 3. Joint 速度图
%
% Notes
% -----
% - 不做控制，只看规划出的参考轨迹
% - 机器人为平面 2R
% - 使用解析 IK，并采用"邻近上一时刻解"策略保持分支连续
% - 对于逐点 IK 得到的轨迹，关节速度使用数值差分估计
%
% Dependencies
% ------------
% - Peter Corke Robotics Toolbox
% - Link, SerialLink, jtraj, lspb

clear;
clc;
close all;

%% 1. 建立平面 2R 机器人
l1 = 1.0;
l2 = 0.8;

L1 = Link('d', 0, 'a', l1, 'alpha', 0);
L2 = Link('d', 0, 'a', l2, 'alpha', 0);
robot = SerialLink([L1 L2], 'name', 'planar2R');

%% 2. 定义笛卡尔 waypoints
cart_points = [
    1.40  0.20;
    1.20  0.80;
    0.85  1.05;
    0.60  0.55;
    1.00  0.10
];

n_wp = size(cart_points, 1);

%% 3. 检查点是否可达
for i = 1:n_wp
    x = cart_points(i,1);
    y = cart_points(i,2);
    r = sqrt(x^2 + y^2);

    if r > (l1 + l2) || r < abs(l1 - l2)
        error('第 %d 个笛卡尔点 [%.3f, %.3f] 不可达。', i, x, y);
    end
end

%% 4. 先计算离散的 joint waypoints（供 Route 1 / 3 使用）
joint_waypoints = zeros(n_wp, 2);
joint_waypoints(1,:) = ik_2R_continuous(cart_points(1,:), l1, l2, []);

for i = 2:n_wp
    joint_waypoints(i,:) = ik_2R_continuous(cart_points(i,:), l1, l2, joint_waypoints(i-1,:));
end

%% 5. 通用参数
N_seg = 50;        % 每段采样点数
dt = 0.05;         % 统一采样周期 [s]

%% =========================
% Route 1: IK -> joint-space jtraj
% ==========================
q_path_1 = [];
qd_path_1 = [];

for i = 1:n_wp-1
    q0 = joint_waypoints(i,:);
    q1 = joint_waypoints(i+1,:);

    [q_seg, qd_seg, ~] = jtraj(q0, q1, N_seg);

    % jtraj 默认内部时间归一化到 [0,1]
    % 若外部采样周期为 dt，则速度需要缩放为 /((N_seg-1)*dt)
    T_seg = (N_seg - 1) * dt;
    qd_seg = qd_seg / T_seg;

    if i > 1
        q_seg  = q_seg(2:end,:);
        qd_seg = qd_seg(2:end,:);
    end

    q_path_1  = [q_path_1;  q_seg];
    qd_path_1 = [qd_path_1; qd_seg];
end

p_path_1 = fkine_path_2d(robot, q_path_1);

%% =========================
% Route 2: Cartesian linear interpolation -> IK
% ==========================
p_path_2 = [];

for i = 1:n_wp-1
    p0 = cart_points(i,:);
    p1 = cart_points(i+1,:);

    alpha = linspace(0, 1, N_seg)';
    p_seg = (1 - alpha).*p0 + alpha.*p1;

    if i > 1
        p_seg = p_seg(2:end,:);
    end

    p_path_2 = [p_path_2; p_seg];
end

q_path_2 = ik_path_continuous(p_path_2, l1, l2);
qd_path_2 = numerical_diff(q_path_2, dt);

% 用 FK 再验证一次实际几何路径
p_path_2_check = fkine_path_2d(robot, q_path_2);

%% =========================
% Route 3: IK -> joint-space LSPB
% ==========================
q_path_3 = [];
qd_path_3 = [];

for i = 1:n_wp-1
    q0 = joint_waypoints(i,:);
    q1 = joint_waypoints(i+1,:);

    [s, sd, ~] = lspb(0, 1, N_seg);

    q_seg = zeros(N_seg, 2);
    qd_seg = zeros(N_seg, 2);

    delta_q = q1 - q0;
    T_seg = (N_seg - 1) * dt;

    for k = 1:N_seg
        q_seg(k,:)  = q0 + s(k)  * delta_q;
        qd_seg(k,:) = (sd(k) / T_seg) * delta_q;
    end

    if i > 1
        q_seg  = q_seg(2:end,:);
        qd_seg = qd_seg(2:end,:);
    end

    q_path_3  = [q_path_3;  q_seg];
    qd_path_3 = [qd_path_3; qd_seg];
end

p_path_3 = fkine_path_2d(robot, q_path_3);

%% =========================
% Route 4: Cartesian LSPB interpolation -> IK
% ==========================
p_path_4 = [];

for i = 1:n_wp-1
    p0 = cart_points(i,:);
    p1 = cart_points(i+1,:);

    [s, ~, ~] = lspb(0, 1, N_seg);

    p_seg = zeros(N_seg, 2);
    delta_p = p1 - p0;

    for k = 1:N_seg
        p_seg(k,:) = p0 + s(k) * delta_p;
    end

    if i > 1
        p_seg = p_seg(2:end,:);
    end

    p_path_4 = [p_path_4; p_seg];
end

q_path_4 = ik_path_continuous(p_path_4, l1, l2);
qd_path_4 = numerical_diff(q_path_4, dt);

% FK 验证
p_path_4_check = fkine_path_2d(robot, q_path_4);

%% 6. 时间轴
t1 = (0:size(q_path_1,1)-1)' * dt;
t2 = (0:size(q_path_2,1)-1)' * dt;
t3 = (0:size(q_path_3,1)-1)' * dt;
t4 = (0:size(q_path_4,1)-1)' * dt;

%% 7. 笛卡尔路径对比图
figure('Name','Cartesian Path Comparison');
plot(cart_points(:,1), cart_points(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8); hold on;
plot(p_path_1(:,1), p_path_1(:,2), '-',  'LineWidth', 2.0);
plot(p_path_2_check(:,1), p_path_2_check(:,2), '--', 'LineWidth', 2.0);
plot(p_path_3(:,1), p_path_3(:,2), '-.', 'LineWidth', 2.0);
plot(p_path_4_check(:,1), p_path_4_check(:,2), ':',  'LineWidth', 2.5);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Cartesian Path Comparison');
legend('Waypoints', ...
       'Route 1: IK -> joint jtraj', ...
       'Route 2: Cartesian linear -> IK', ...
       'Route 3: IK -> joint LSPB', ...
       'Route 4: Cartesian LSPB -> IK', ...
       'Location', 'best');

%% 8. Route 2 几何验证
figure('Name','Route 2 Cartesian Verification');
plot(p_path_2(:,1), p_path_2(:,2), '--', 'LineWidth', 2.0); hold on;
plot(p_path_2_check(:,1), p_path_2_check(:,2), '-', 'LineWidth', 1.5);
plot(cart_points(:,1), cart_points(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Route 2: Planned Cartesian Path vs FK-verified Path');
legend('Planned Cartesian Path', 'FK-verified Path', 'Waypoints', 'Location', 'best');

%% 9. Route 4 几何验证
figure('Name','Route 4 Cartesian Verification');
plot(p_path_4(:,1), p_path_4(:,2), '--', 'LineWidth', 2.0); hold on;
plot(p_path_4_check(:,1), p_path_4_check(:,2), '-', 'LineWidth', 1.5);
plot(cart_points(:,1), cart_points(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Route 4: Planned Cartesian Path vs FK-verified Path');
legend('Planned Cartesian Path', 'FK-verified Path', 'Waypoints', 'Location', 'best');

%% 10. Joint 位置图
figure('Name','Joint Position Comparison');

subplot(2,1,1);
plot(t1, q_path_1(:,1), 'LineWidth', 1.5); hold on;
plot(t2, q_path_2(:,1), 'LineWidth', 1.5);
plot(t3, q_path_3(:,1), 'LineWidth', 1.5);
plot(t4, q_path_4(:,1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('Joint 1 Position');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

subplot(2,1,2);
plot(t1, q_path_1(:,2), 'LineWidth', 1.5); hold on;
plot(t2, q_path_2(:,2), 'LineWidth', 1.5);
plot(t3, q_path_3(:,2), 'LineWidth', 1.5);
plot(t4, q_path_4(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('q_2 (rad)');
title('Joint 2 Position');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

%% 11. Joint 速度图
figure('Name','Joint Velocity Comparison');

subplot(2,1,1);
plot(t1, qd_path_1(:,1), 'LineWidth', 1.5); hold on;
plot(t2, qd_path_2(:,1), 'LineWidth', 1.5);
plot(t3, qd_path_3(:,1), 'LineWidth', 1.5);
plot(t4, qd_path_4(:,1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('dq_1 (rad/s)');
title('Joint 1 Velocity');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

subplot(2,1,2);
plot(t1, qd_path_1(:,2), 'LineWidth', 1.5); hold on;
plot(t2, qd_path_2(:,2), 'LineWidth', 1.5);
plot(t3, qd_path_3(:,2), 'LineWidth', 1.5);
plot(t4, qd_path_4(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('dq_2 (rad/s)');
title('Joint 2 Velocity');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

%% 12. Cartesian 位置-时间图
figure('Name','Cartesian Position vs Time');

subplot(2,1,1);
plot(t1, p_path_1(:,1), 'LineWidth', 1.5); hold on;
plot(t2, p_path_2_check(:,1), 'LineWidth', 1.5);
plot(t3, p_path_3(:,1), 'LineWidth', 1.5);
plot(t4, p_path_4_check(:,1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('X');
title('Cartesian X Position');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

subplot(2,1,2);
plot(t1, p_path_1(:,2), 'LineWidth', 1.5); hold on;
plot(t2, p_path_2_check(:,2), 'LineWidth', 1.5);
plot(t3, p_path_3(:,2), 'LineWidth', 1.5);
plot(t4, p_path_4_check(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Y');
title('Cartesian Y Position');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

%% 13. 计算 Cartesian 速度（数值差分）
vd_path_1 = numerical_diff(p_path_1, dt);
vd_path_2 = numerical_diff(p_path_2_check, dt);
vd_path_3 = numerical_diff(p_path_3, dt);
vd_path_4 = numerical_diff(p_path_4_check, dt);

%% 14. Cartesian 速度-时间图
figure('Name','Cartesian Velocity vs Time');

subplot(2,1,1);
plot(t1, vd_path_1(:,1), 'LineWidth', 1.5); hold on;
plot(t2, vd_path_2(:,1), 'LineWidth', 1.5);
plot(t3, vd_path_3(:,1), 'LineWidth', 1.5);
plot(t4, vd_path_4(:,1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('V_x');
title('Cartesian X Velocity');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

subplot(2,1,2);
plot(t1, vd_path_1(:,2), 'LineWidth', 1.5); hold on;
plot(t2, vd_path_2(:,2), 'LineWidth', 1.5);
plot(t3, vd_path_3(:,2), 'LineWidth', 1.5);
plot(t4, vd_path_4(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('V_y');
title('Cartesian Y Velocity');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

%% 15. Cartesian 速度模长图
speed_1 = vecnorm(vd_path_1, 2, 2);
speed_2 = vecnorm(vd_path_2, 2, 2);
speed_3 = vecnorm(vd_path_3, 2, 2);
speed_4 = vecnorm(vd_path_4, 2, 2);

figure('Name','Cartesian Speed Magnitude');
plot(t1, speed_1, 'LineWidth', 1.5); hold on;
plot(t2, speed_2, 'LineWidth', 1.5);
plot(t3, speed_3, 'LineWidth', 1.5);
plot(t4, speed_4, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('|v|');
title('Cartesian Speed Magnitude');
legend('Route 1', 'Route 2', 'Route 3', 'Route 4', 'Location', 'best');

% %% 16. 可选动画
% figure('Name', 'Animation - Route 1');
% robot.plot(q_path_1);
% 
% figure('Name', 'Animation - Route 2');
% robot.plot(q_path_2);
% 
% figure('Name', 'Animation - Route 3');
% robot.plot(q_path_3);
% 
% figure('Name', 'Animation - Route 4');
% robot.plot(q_path_4);

%% ===== 本地函数 =====

function p_path = fkine_path_2d(robot, q_path)
%FKINE_PATH_2D 对关节轨迹做正运动学，提取末端 XY 位置
%
% Parameters
% ----------
% robot : SerialLink
%     机器人模型
% q_path : (N,2) double
%     关节轨迹
%
% Returns
% -------
% p_path : (N,2) double
%     末端 XY 轨迹

    N = size(q_path, 1);
    p_path = zeros(N, 2);

    for k = 1:N
        T = robot.fkine(q_path(k,:));
        p_path(k,:) = T.t(1:2)';
    end
end

function q_path = ik_path_continuous(p_path, l1, l2)
%IK_PATH_CONTINUOUS 对整条笛卡尔路径做连续逆运动学
%
% Parameters
% ----------
% p_path : (N,2) double
%     笛卡尔路径
% l1 : double
%     第一连杆长度
% l2 : double
%     第二连杆长度
%
% Returns
% -------
% q_path : (N,2) double
%     连续选择分支后的关节轨迹

    N = size(p_path, 1);
    q_path = zeros(N, 2);

    q_path(1,:) = ik_2R_continuous(p_path(1,:), l1, l2, []);

    for k = 2:N
        q_path(k,:) = ik_2R_continuous(p_path(k,:), l1, l2, q_path(k-1,:));
    end
end

function q = ik_2R_continuous(p, l1, l2, q_prev)
%IK_2R_CONTINUOUS 平面 2R 机械臂解析逆运动学，并优先选择与上一时刻更接近的解
%
% Parameters
% ----------
% p : (1,2) double
%     目标笛卡尔位置 [x, y]
% l1 : double
%     第一连杆长度
% l2 : double
%     第二连杆长度
% q_prev : (1,2) double or []
%     上一个时刻的关节解；若为空，则默认返回第一组可行解
%
% Returns
% -------
% q : (1,2) double
%     选中的关节解 [q1, q2]

    x = p(1);
    y = p(2);

    c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
    c2 = min(max(c2, -1), 1);

    s2_pos = sqrt(1 - c2^2);
    s2_neg = -sqrt(1 - c2^2);

    q2_a = atan2(s2_pos, c2);
    q2_b = atan2(s2_neg, c2);

    q1_a = atan2(y, x) - atan2(l2*sin(q2_a), l1 + l2*cos(q2_a));
    q1_b = atan2(y, x) - atan2(l2*sin(q2_b), l1 + l2*cos(q2_b));

    qa = [wrapToPiLocal(q1_a), wrapToPiLocal(q2_a)];
    qb = [wrapToPiLocal(q1_b), wrapToPiLocal(q2_b)];

    if isempty(q_prev)
        q = qa;
    else
        da = norm(angleDiffVec(qa, q_prev));
        db = norm(angleDiffVec(qb, q_prev));

        if da <= db
            q = qa;
        else
            q = qb;
        end
    end
end

function d = angleDiffVec(a, b)
%ANGLEDIFFVEC 计算两个关节角向量的最小角差
    d = [wrapToPiLocal(a(1)-b(1)), wrapToPiLocal(a(2)-b(2))];
end

function a = wrapToPiLocal(a)
%WRAPTOPILOCAL 将角度包裹到 [-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end

function qd = numerical_diff(q, dt)
%NUMERICAL_DIFF 对离散关节轨迹做数值微分
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
% qd : (N,m) double
%     数值估计速度

    qd = zeros(size(q));

    if size(q,1) < 2
        return;
    end

    % 前向差分 / 后向差分 / 中心差分
    qd(1,:) = (q(2,:) - q(1,:)) / dt;

    for k = 2:size(q,1)-1
        qd(k,:) = (q(k+1,:) - q(k-1,:)) / (2*dt);
    end

    qd(end,:) = (q(end,:) - q(end-1,:)) / dt;
end