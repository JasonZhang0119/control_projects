%% demo_planar_2R_multi_waypoints
% 多路径点 + 闭环控制 + 分析图

clear;
clc;
close all;

%% 1. 构建 2R 机器人
l1 = 1.0;
l2 = 0.8;

L1 = Link('d', 0, 'a', l1, 'alpha', 0);
L2 = Link('d', 0, 'a', l2, 'alpha', 0);

robot = SerialLink([L1 L2], 'name', 'planar2R');

%% 2. 定义多个 waypoints（关键修改点）
% 每一行是一个 q 点
waypoints = [
    0.2   0.3;
    0.8  -0.5;
    1.2   0.6;
    0.5  -1.0;
    1.0  -0.2
];

n_wp = size(waypoints,1);

%% 3. 生成多段轨迹（拼接 jtraj）
N_seg = 40;   % 每段轨迹点数

q_ref_log = [];

for i = 1:n_wp-1
    q_start = waypoints(i,:);
    q_end   = waypoints(i+1,:);
    
    [q_seg, ~, ~] = jtraj(q_start, q_end, N_seg);
    
    % 避免重复拼接点（去掉每段第一个点）
    if i > 1
        q_seg = q_seg(2:end,:);
    end
    
    q_ref_log = [q_ref_log; q_seg];
end

N = size(q_ref_log,1);

dt = 0.05;
t = (0:N-1)' * dt;

%% 4. 初始化日志
q_act_log = zeros(N,2);
e_q_log   = zeros(N,2);

p_ref_log = zeros(N,2);
p_act_log = zeros(N,2);
e_p_log   = zeros(N,2);

%% 5. 控制器
Kp = diag([4,4]);

q = q_ref_log(1,:)';   % 初始状态

%% 6. 主循环
for k = 1:N
    
    q_ref = q_ref_log(k,:)';
    
    % 误差
    e_q = q_ref - q;
    
    % P 控制
    dq = Kp * e_q;
    
    % 更新
    q = q + dq * dt;
    
    % 记录
    q_act_log(k,:) = q';
    e_q_log(k,:)   = e_q';
    
    % 参考末端
    T_ref = robot.fkine(q_ref');
    p_ref = T_ref.t(1:2)';
    p_ref_log(k,:) = p_ref;
    
    % 实际末端
    T_act = robot.fkine(q');
    p_act = T_act.t(1:2)';
    p_act_log(k,:) = p_act;
    
    % task error
    e_p_log(k,:) = p_ref - p_act;
end

%% 7. 动画
figure('Name','Animation');
robot.plot(q_act_log);

%% 8. Joint tracking
figure('Name','Joint Tracking');

for i = 1:2
    subplot(2,1,i);
    plot(t, q_ref_log(:,i),'--','LineWidth',1.5); hold on;
    plot(t, q_act_log(:,i),'LineWidth',1.5);
    grid on;
    ylabel(['q_',num2str(i)]);
end
xlabel('Time');

%% 9. Joint error
figure('Name','Joint Error');

for i = 1:2
    subplot(2,1,i);
    plot(t, e_q_log(:,i),'LineWidth',1.5);
    grid on;
    ylabel(['e_',num2str(i)]);
end
xlabel('Time');

%% 10. Task trajectory
figure('Name','End-effector Trajectory');

plot(p_ref_log(:,1), p_ref_log(:,2),'--','LineWidth',2); hold on;
plot(p_act_log(:,1), p_act_log(:,2),'LineWidth',2);

% 标记 waypoints
for i = 1:n_wp
    T_wp = robot.fkine(waypoints(i,:));
    p_wp = T_wp.t(1:2)';
    plot(p_wp(1), p_wp(2),'o','MarkerSize',8,'LineWidth',2);
end

grid on;
axis equal;
xlabel('X'); ylabel('Y');
legend('ref','actual','waypoints');

%% 11. Task error
figure('Name','Task Error');

subplot(2,1,1);
plot(t, e_p_log(:,1),'LineWidth',1.5); grid on;
title('X error');

subplot(2,1,2);
plot(t, e_p_log(:,2),'LineWidth',1.5); grid on;
title('Y error');