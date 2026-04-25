function Animate2R(q_traj, param, options)
%ANIMATE2R 平面 2R 机械臂轨迹动画
%
% 参数
% ----------
% q_traj : double, size (2, N)
%     关节轨迹序列。
%     每一列为一个时刻的关节角：
%         q_traj(:, k) = [q1(k); q2(k)]
%
% param : struct
%     2R 机械臂参数结构体。
%
% options : struct, 可选
%     动画配置项：
%         dt          : 每帧暂停时间，默认 0.02
%         show_trace  : 是否显示末端轨迹，默认 true
%         axis_limit  : 坐标轴范围，默认根据 l1 + l2 自动设置
%         title       : 图标题，默认 '2R Robot Animation'

    if nargin < 3 || isempty(options)
        options = struct();
    end

    if ~isfield(options, 'dt')
        options.dt = 0.02;
    end

    if ~isfield(options, 'show_trace')
        options.show_trace = true;
    end

    if ~isfield(options, 'title_text')
        options.title_text = '2R Robot Animation';
    end

    if size(q_traj, 1) ~= 2
        error('Animate2R:DimensionMismatch', ...
              'q_traj 必须为 2 x N 矩阵。');
    end

    N = size(q_traj, 2);
    ee_traj = zeros(2, N);

    h = struct();

    figure('Name', options.title_text);

    for k = 1:N
        q = q_traj(:, k);

        p = FK2R_Analytic(q, param);
        ee_traj(:, k) = p;

        plot_options = struct();
        plot_options.title_text = sprintf('%s | Frame %d / %d', ...
            options.title_text, k, N);
        plot_options.show_frame = true;

        if isfield(options, 'axis_limit')
            plot_options.axis_limit = options.axis_limit;
        end

        if options.show_trace
            plot_options.ee_traj = ee_traj(:, 1:k);
        end

        h = Plot2RRobot(q, param, plot_options, h);

        pause(options.dt);
    end
end