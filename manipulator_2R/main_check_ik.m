%% main_check_ik.m
%MAIN_CHECK_IK 测试 2R 机械臂解析 IK、位置数值 IK 和 SE(3) 数值 IK
%
% Notes
% -----
% - 需要已有函数：
%   Param2R
%   FK_DH
%   JacobianFromTAll
%   IK2R_Analytic
%   IKPosition_Numerical
%   IKSE3_Numerical
% - 默认 Jacobian convention:
%   J = [Jw; Jv]
%   V = [omega; v]

clear;
clc;
close all;

%% ================================================================
% 1. 创建机器人参数
% ================================================================
method = 'standard';
param = Param2R(method);

%% ================================================================
% 2. 设置测试目标
% ================================================================
p_d = [1.0; 0.8];              % 2R 平面位置目标 [x; y]
% q0  = deg2rad([20; 20]);       % 初始猜测
q0 = deg2rad([1; 89]);

fprintf('==============================\n');
fprintf('2R IK Test\n');
fprintf('DH method: %s\n', param.dh_method);
fprintf('Target p_d = [%.6f, %.6f]^T\n', p_d(1), p_d(2));
fprintf('Initial q0 = [%.6f, %.6f] deg\n', rad2deg(q0(1)), rad2deg(q0(2)));
fprintf('==============================\n\n');

%% ================================================================
% 3. 测试解析 IK
% ================================================================
fprintf('---- Test 1: IK2R_Analytic ----\n');

try
    q_all = IK2R_Analytic(p_d, param);

    fprintf('Analytic IK solution(s) [deg]:\n');
    disp(rad2deg(q_all));

    % 兼容两种常见输出：
    % 1. q_all 是 2 x num_solutions
    % 2. q_all 是 num_solutions x 2
    if size(q_all, 1) == 2
        q_analytic = q_all(:, 1);
    elseif size(q_all, 2) == 2
        q_analytic = q_all(1, :).';
    else
        error('main_check_ik:InvalidAnalyticIKOutput', ...
              'IK2R_Analytic 输出应为 2 x N 或 N x 2。');
    end

    p_analytic = get_tool_position(q_analytic, param, 2);
    err_analytic = norm(p_d - p_analytic);

    fprintf('Selected analytic q [deg] = [%.6f, %.6f]\n', ...
        rad2deg(q_analytic(1)), rad2deg(q_analytic(2)));
    fprintf('Analytic FK p = [%.6f, %.6f]\n', ...
        p_analytic(1), p_analytic(2));
    fprintf('Analytic position error = %.6e\n\n', err_analytic);

catch ME
    warning('Analytic IK test failed: %s', ME.message);
    q_analytic = [];
end

%% ================================================================
% 4. 测试位置数值 IK
% ================================================================
fprintf('---- Test 2: IKPosition_Numerical ----\n');

options_pos = struct();
options_pos.max_iter = 100;
options_pos.tol = 1e-8;
options_pos.alpha = 0.5;

[q_pos, info_pos] = IKPosition_Numerical(p_d, q0, param, options_pos);

p_pos = get_tool_position(q_pos, param, 2);
err_pos = norm(p_d - p_pos);

fprintf('Position IK converged = %d\n', info_pos.converged);
fprintf('Position IK iter = %d\n', info_pos.iter);
fprintf('Position IK q [deg] = [%.6f, %.6f]\n', ...
    rad2deg(q_pos(1)), rad2deg(q_pos(2)));
fprintf('Position IK FK p = [%.6f, %.6f]\n', ...
    p_pos(1), p_pos(2));
fprintf('Position IK final error = %.6e\n\n', err_pos);

%% ================================================================
% 5. 测试 SE(3) 数值 IK
% ================================================================
fprintf('---- Test 3: IKSE3_Numerical ----\n');

% 对 2R 来说，完整 SE(3) 任务是过约束的。
% 因此这里用一个已知可达 q_ref 通过 FK 生成 T_sd。
% 这样 IKSE3_Numerical 的目标一定可达。
if ~isempty(q_analytic)
    q_ref = q_analytic;
else
    q_ref = q_pos;
end

T_sd = get_tool_transform(q_ref, param);

options_se3 = struct();
options_se3.max_iter = 100;
options_se3.tol_pos = 1e-8;
options_se3.tol_rot = 1e-8;
options_se3.alpha = 1;
options_se3.lambda = 1e-3;

options_se3.frame = 'space';

[q_se3, info_se3] = IKSE3_Numerical(T_sd, q0, param, options_se3);

T_se3 = get_tool_transform(q_se3, param);
p_se3 = T_se3(1:2, 4);
err_se3_pos = norm(p_d - p_se3);

fprintf('SE(3) IK converged = %d\n', info_se3.converged);
fprintf('SE(3) IK iter = %d\n', info_se3.iter);
fprintf('SE(3) IK q [deg] = [%.6f, %.6f]\n', ...
    rad2deg(q_se3(1)), rad2deg(q_se3(2)));
fprintf('SE(3) IK FK p = [%.6f, %.6f]\n', ...
    p_se3(1), p_se3(2));
fprintf('SE(3) IK position error = %.6e\n', err_se3_pos);
fprintf('SE(3) IK final SE(3) error norm = %.6e\n\n', ...
    info_se3.final_error_norm);

%% ================================================================
% 6. 汇总
% ================================================================
fprintf('==============================\n');
fprintf('Summary\n');
fprintf('==============================\n');

if ~isempty(q_analytic)
    fprintf('Analytic q [deg]      = [%.6f, %.6f]\n', ...
        rad2deg(q_analytic(1)), rad2deg(q_analytic(2)));
end

fprintf('Position IK q [deg]   = [%.6f, %.6f]\n', ...
    rad2deg(q_pos(1)), rad2deg(q_pos(2)));

fprintf('SE(3) IK q [deg]      = [%.6f, %.6f]\n', ...
    rad2deg(q_se3(1)), rad2deg(q_se3(2)));

%% ================================================================
% 7. 绘制收敛曲线
% ================================================================
figure;
semilogy(info_pos.err_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Iteration');
ylabel('Position error norm');
title('IKPosition\_Numerical Convergence');

figure;
semilogy(info_se3.err_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Iteration');
ylabel('SE(3) error norm');
title('IKSE3\_Numerical Convergence');

%% ================================================================
% 8. 绘制机械臂构型
% ================================================================
figure;
hold on;
grid on;
axis equal;

plot_2r(q0, param, '--o', 'Initial');
plot_2r(q_pos, param, '-o', 'Position IK');
plot_2r(q_se3, param, '-s', 'SE(3) IK');

if ~isempty(q_analytic)
    plot_2r(q_analytic, param, '-d', 'Analytic IK');
end

plot(p_d(1), p_d(2), 'x', ...
    'MarkerSize', 12, ...
    'LineWidth', 2, ...
    'DisplayName', 'Target');

xlabel('x');
ylabel('y');
title('2R IK Result Comparison');
legend('Location', 'best');

%% ================================================================
% Local helper functions
% ================================================================

function T_tool = get_tool_transform(q, param)
%GET_TOOL_TRANSFORM 获取 base 到 tool 的齐次变换矩阵
%
% Parameters
% ----------
% q : double, size (n, 1)
%     当前关节角。
%
% param : struct
%     机器人参数结构体。
%
% Returns
% -------
% T_tool : double, size (4, 4)
%     base 到 tool 的齐次变换矩阵。

    T_all = FK_DH(q, param.dh_table, param.joint_types, param.dh_method);
    T_tool = T_all(:, :, end) * param.T_tool;
end


function p = get_tool_position(q, param, dim)
%GET_TOOL_POSITION 获取 tool 位置
%
% Parameters
% ----------
% q : double, size (n, 1)
%     当前关节角。
%
% param : struct
%     机器人参数结构体。
%
% dim : double
%     返回位置维度，2 或 3。
%
% Returns
% -------
% p : double, size (dim, 1)
%     tool 位置。

    T_tool = get_tool_transform(q, param);
    p = T_tool(1:dim, 4);
end


function plot_2r(q, param, line_spec, display_name)
%PLOT_2R 绘制 2R 平面机械臂
%
% Parameters
% ----------
% q : double, size (2, 1)
%     当前关节角。
%
% param : struct
%     2R 参数结构体。
%
% line_spec : char
%     MATLAB 绘图线型。
%
% display_name : char 或 string
%     图例名称。

    q1 = q(1);
    q2 = q(2);

    l1 = param.l1;
    l2 = param.l2;

    p0 = [0; 0];

    p1 = [
        l1 * cos(q1);
        l1 * sin(q1)
    ];

    p2 = [
        l1 * cos(q1) + l2 * cos(q1 + q2);
        l1 * sin(q1) + l2 * sin(q1 + q2)
    ];

    plot([p0(1), p1(1), p2(1)], ...
         [p0(2), p1(2), p2(2)], ...
         line_spec, ...
         'LineWidth', 1.5, ...
         'DisplayName', display_name);
end