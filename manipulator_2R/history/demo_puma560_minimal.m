%% demo_puma560_minimal_fixed
% PUMA560 最小演示（修正版）
%
% 1. 加载机器人模型
% 2. 定义起点和终点关节角
% 3. 生成 joint-space 轨迹
% 4. 单独播放动画
% 5. 计算并绘制末端位置轨迹
% 6. 单独绘制关节轨迹
%
% Notes
% -----
% - 避免在多个 subplot 中复用 p560.plot 造成显示被覆盖。
% - 当前重点是跑通最小链路，而不是做复杂控制。

clear;
clc;
close all;

%% 1. 加载模型
mdl_puma560;

%% 2. 定义起点与终点
q_start = [0 0 0 0 0 0];
q_goal  = [0 pi/6 -pi/4 0 pi/6 0];

disp('q_start = ');
disp(q_start);

disp('q_goal = ');
disp(q_goal);

%% 3. 打印起点与终点末端位姿
T_start = p560.fkine(q_start);
T_goal  = p560.fkine(q_goal);

disp('T_start = ');
disp(T_start);

disp('T_goal = ');
disp(T_goal);

disp('p_start = ');
disp(transl(T_start));

disp('p_goal = ');
disp(transl(T_goal));

%% 4. 生成关节空间轨迹
n_step = 80;
[q_traj, qd_traj, qdd_traj] = jtraj(q_start, q_goal, n_step);

%% 5. 单独显示起点姿态
figure('Name', 'Start Pose');
p560.plot(q_start);
title('Start Pose');

%% 6. 单独显示终点姿态
figure('Name', 'Goal Pose');
p560.plot(q_goal);
title('Goal Pose');

%% 7. 单独播放动画
figure('Name', 'Joint-space Trajectory Animation');
p560.plot(q_traj);

%% 8. 计算末端位置轨迹
p_xyz = zeros(n_step, 3);

for k = 1:n_step
    T_k = p560.fkine(q_traj(k, :));
    p_xyz(k, :) = transl(T_k);
end

%% 9. 绘制末端三维轨迹
figure('Name', 'End-effector Position Trajectory');
plot3(p_xyz(:,1), p_xyz(:,2), p_xyz(:,3), 'LineWidth', 2);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End-effector Position Trajectory');

hold on;
plot3(p_xyz(1,1), p_xyz(1,2), p_xyz(1,3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
plot3(p_xyz(end,1), p_xyz(end,2), p_xyz(end,3), 's', 'MarkerSize', 8, 'LineWidth', 2);
legend('Trajectory', 'Start', 'Goal');

%% 10. 绘制关节轨迹
figure('Name', 'All Joint-space Trajectories');
plot(q_traj, 'LineWidth', 1.5);
grid on;
xlabel('Step');
ylabel('Joint Position (rad)');
title('All Joint-space Trajectories');
legend('q1','q2','q3','q4','q5','q6');