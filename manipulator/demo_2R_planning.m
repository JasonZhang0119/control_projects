%% demo_planar_2R_compare_planning
% 比较两种规划方式：
% 1. 笛卡尔 waypoints -> IK -> joint-space 规划
% 2. 笛卡尔 waypoints -> Cartesian 直接插值 -> 逐点 IK
%
% Notes
% -----
% - 不做控制，只看规划出来的参考路径
% - 机械臂模型为平面 2R
% - 使用解析逆解，并用"邻近上一时刻解"的方式保持 IK 连续性

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

%% 4. 路线 1：先 IK 求离散关节点，再做 joint-space 轨迹拼接
joint_waypoints = zeros(n_wp, 2);

% 先对第一个点取一个固定分支（肘下/肘上都可以，这里交给解析函数）
joint_waypoints(1,:) = ik_2R_continuous(cart_points(1,:), l1, l2, []);

for i = 2:n_wp
    joint_waypoints(i,:) = ik_2R_continuous(cart_points(i,:), l1, l2, joint_waypoints(i-1,:));
end

N_seg = 40;
q_path_1 = [];

for i = 1:n_wp-1
    q0 = joint_waypoints(i,:);
    q1 = joint_waypoints(i+1,:);
    [q_seg, ~, ~] = jtraj(q0, q1, N_seg);

    if i > 1
        q_seg = q_seg(2:end,:);
    end

    q_path_1 = [q_path_1; q_seg];
end

N1 = size(q_path_1,1);
p_path_1 = zeros(N1, 2);

for k = 1:N1
    T = robot.fkine(q_path_1(k,:));
    p_path_1(k,:) = T.t(1:2)';
end

%% 5. 路线 2：先对笛卡尔点做分段直线插值，再逐点 IK
P_seg = 40;
p_path_2 = [];

for i = 1:n_wp-1
    p0 = cart_points(i,:);
    p1 = cart_points(i+1,:);

    alpha = linspace(0, 1, P_seg)';
    p_seg = (1-alpha).*p0 + alpha.*p1;

    if i > 1
        p_seg = p_seg(2:end,:);
    end

    p_path_2 = [p_path_2; p_seg];
end

N2 = size(p_path_2,1);
q_path_2 = zeros(N2, 2);

q_path_2(1,:) = ik_2R_continuous(p_path_2(1,:), l1, l2, []);

for k = 2:N2
    q_path_2(k,:) = ik_2R_continuous(p_path_2(k,:), l1, l2, q_path_2(k-1,:));
end

% 再正运动学验证一遍
p_path_2_check = zeros(N2, 2);
for k = 1:N2
    T = robot.fkine(q_path_2(k,:));
    p_path_2_check(k,:) = T.t(1:2)';
end

%% 6. 可视化：笛卡尔路径对比
figure('Name','Cartesian Path Comparison');
plot(cart_points(:,1), cart_points(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8); hold on;
plot(p_path_1(:,1), p_path_1(:,2), '-', 'LineWidth', 2.0);
plot(p_path_2(:,1), p_path_2(:,2), '--', 'LineWidth', 2.0);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Cartesian Path Comparison');
legend('Waypoints', 'Route 1: IK -> Joint Planning', 'Route 2: Cartesian Planning -> IK', ...
    'Location', 'best');

%% 7. 可视化：路线 2 的 IK 验证
figure('Name','Route 2 Cartesian Planning Verification');
plot(p_path_2(:,1), p_path_2(:,2), '--', 'LineWidth', 2.0); hold on;
plot(p_path_2_check(:,1), p_path_2_check(:,2), '-', 'LineWidth', 1.5);
plot(cart_points(:,1), cart_points(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
title('Route 2 Planned Cartesian Path vs FK-verified Path');
legend('Planned Cartesian Path', 'FK-verified Cartesian Path', 'Waypoints', ...
    'Location', 'best');

%% 8. 可视化：两条路线的 joint 轨迹
figure('Name','Joint Trajectories Comparison');

subplot(2,2,1);
plot(q_path_1(:,1), 'LineWidth', 1.6);
grid on;
title('Route 1: q_1');
xlabel('Step');
ylabel('rad');

subplot(2,2,2);
plot(q_path_1(:,2), 'LineWidth', 1.6);
grid on;
title('Route 1: q_2');
xlabel('Step');
ylabel('rad');

subplot(2,2,3);
plot(q_path_2(:,1), 'LineWidth', 1.6);
grid on;
title('Route 2: q_1');
xlabel('Step');
ylabel('rad');

subplot(2,2,4);
plot(q_path_2(:,2), 'LineWidth', 1.6);
grid on;
title('Route 2: q_2');
xlabel('Step');
ylabel('rad');

%% 9. 动画：路线 1
figure('Name', 'Animation - Route 1');
robot.plot(q_path_1);

%% 10. 动画：路线 2
figure('Name', 'Animation - Route 2');
robot.plot(q_path_2);
