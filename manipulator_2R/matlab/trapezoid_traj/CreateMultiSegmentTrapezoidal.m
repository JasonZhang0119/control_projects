function traj = CreateMultiSegmentTrapezoidal(q_waypoints, t_waypoints, options)
%CREATEMULTISEGMENTTRAPEZOIDAL 多段梯形速度轨迹生成模板。
%
% 这个函数用于生成分段梯形速度轨迹的结构体。
% 当前版本只保留整体框架，并预留两种实现方式：
%   1) zero      : 所有 waypoint 速度为 0
%   2) solve_acc : 按给定时间反解加速度/时间参数
%
% 输入
% ----
% q_waypoints : double, size (n, M)
%     路径点矩阵，每一列表示一个 waypoint。
%
% t_waypoints : double, size (1, M) or (M, 1)
%     严格递增的 waypoint 时间序列。
%
% options : struct, optional
%     可选字段建议：
%         a_max            : 最大加速度
%         stop_at_waypoints : 是否在 waypoint 处停住，默认 true
%         velocity_mode     : 'zero' | 'solve_acc'
%             zero      : 方法1，所有 waypoint 速度置 0
%             solve_acc : 方法2，设定时间，反解加速度/中间参数
%         qd_waypoints      : 方法2 可选，自定义 waypoint 速度
%         a_waypoints       : 方法2 可选，自定义 waypoint 加速度
%
% 输出
% ----
% traj : struct
%     供后续评估使用的轨迹结构体。
%     建议包含字段：
%         type
%         q_waypoints
%         t_waypoints
%         qd_waypoints
%         segments
%         options

    if nargin < 3
        options = struct();
    end

    if ~isfield(options, 'stop_at_waypoints')
        options.stop_at_waypoints = true;
    end

    if ~isfield(options, 'velocity_mode')
        options.velocity_mode = 'zero';
    end

    if ~isfield(options, 'a_max')
        options.a_max = [];
    end

    if ~isfield(options, 'qd_waypoints')
        options.qd_waypoints = [];
    end

    if ~isfield(options, 'a_waypoints')
        options.a_waypoints = [];
    end

    [n, M] = size(q_waypoints);

    if M < 2
        error('CreateMultiSegmentTrapezoidal:InvalidWaypointNumber', ...
              'At least two waypoints are required.');
    end

    if numel(t_waypoints) ~= M
        error('CreateMultiSegmentTrapezoidal:TimeDimensionMismatch', ...
              'The length of t_waypoints must match the number of waypoints.');
    end

    t_waypoints = t_waypoints(:).';

    if any(diff(t_waypoints) <= 0)
        error('CreateMultiSegmentTrapezoidal:InvalidTimeWaypoints', ...
              't_waypoints must be strictly increasing.');
    end

    % ============================================================
    % Step 1：准备 waypoint 速度 / 加速度
    % ============================================================
    %
    % 这里先把两种方法的入口统一好，具体公式后续再补。
    %
    switch lower(options.velocity_mode)
        case 'zero'
            % 方法1：所有 waypoint 速度为 0
            qd_waypoints = zeros(n, M);
            qa_waypoints = zeros(n, M); 

        case 'solve_acc'
            % 方法2：设定时间，反解加速度或中间段参数
            %
            % TODO:
            %   1. 根据你图里的公式，定义每个段的时间分配
            %   2. 计算 waypoint 速度 qd_waypoints
            %   3. 如有需要，再计算 waypoint 加速度 qa_waypoints
            %
            % 建议先准备好以下变量：
            %   qd_waypoints = ...
            %   qa_waypoints = ...
            %
            qd_waypoints = options.qd_waypoints;
            qa_waypoints = options.a_waypoints; 

            if isempty(qd_waypoints)
                qd_waypoints = zeros(n, M); % TODO: 替换为你的反解结果
            end

        otherwise
            error('CreateMultiSegmentTrapezoidal:UnknownVelocityMode', ...
                  'velocity_mode 必须是 ''zero'' 或 ''solve_acc''。');
    end

    if ~isequal(size(qd_waypoints), [n, M])
        error('CreateMultiSegmentTrapezoidal:VelocitySizeMismatch', ...
              'qd_waypoints 的大小必须与 q_waypoints 一致。');
    end

    % ============================================================
    % Step 2：构造每一段的轮廓参数
    % ============================================================
    %
    % TODO:
    %   对于每一段 k：
    %       q0 = q_waypoints(:, k)
    %       qf = q_waypoints(:, k+1)
    %       t0 = t_waypoints(k)
    %       tf = t_waypoints(k+1)
    %
    %   然后构造一段梯形轮廓结构体，供 EvalTrapezoidSegment 使用。
    %
    segments = cell(1, M-1);

    for k = 1:M-1
        seg = struct();
        seg.t0 = t_waypoints(k);
        seg.tf = t_waypoints(k+1);
        seg.h = seg.tf - seg.t0;
        seg.q0 = q_waypoints(:, k);
        seg.qf = q_waypoints(:, k+1);
        seg.qd0 = qd_waypoints(:, k);
        seg.qdf = qd_waypoints(:, k+1);
        seg.mode = options.velocity_mode;

        % TODO:
        %   方法1时，可直接按 stop-at-waypoints 的逻辑构造 profile。
        %   方法2时，可把每段时间/加速度参数存进去，
        %   供后续 EvalTrapezoidSegment 使用。
        %
        seg.profile = [];

        segments{k} = seg;
    end

    % ============================================================
    % 输出
    % ============================================================
    traj.type = 'multi_trapezoidal';
    traj.q_waypoints = q_waypoints;
    traj.t_waypoints = t_waypoints;
    traj.qd_waypoints = qd_waypoints;
    traj.segments = segments;
    traj.options = options;
end
