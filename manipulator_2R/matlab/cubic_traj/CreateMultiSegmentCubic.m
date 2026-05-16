function traj = CreateMultiSegmentCubic(q_waypoints, t_waypoints, options)
%CREATEMULTISEGMENTCUBIC 多段三次多项式轨迹生成（C1 连续）
%
% 参数
% ----------
% q_waypoints : double, size (n, M)
%     路径点，每一列为一个 waypoint。
%
% t_waypoints : double, size (1, M)
%     每个 waypoint 的时间戳，必须严格递增。
%
% options : struct, 可选
%     配置项：
%         velocity_mode : char
%             速度生成方式：
%                 'zero'    : 首尾为 0，中间为 0
%                 'fd'      : finite difference
%                 'custom'  : 使用 options.qd_waypoints
%
%         qd_waypoints : double, size (n, M)
%             自定义 waypoint 速度（当 velocity_mode = 'custom' 时使用）
%
% 返回
% -------
% traj : struct
%     多段轨迹结构体，包含：
%         q_waypoints
%         qd_waypoints
%         t_waypoints
%         segments : 每一段的参数（cell）

    if nargin < 3
        options = struct();
    end

    if ~isfield(options, 'velocity_mode')
        options.velocity_mode = 'fd';
    end

    [n, M] = size(q_waypoints);

    if numel(t_waypoints) ~= M
        error('CreateMultiSegmentCubic:DimensionMismatch', ...
              't_waypoints 长度必须等于 waypoint 数量。');
    end

    if any(diff(t_waypoints) <= 0)
        error('CreateMultiSegmentCubic:InvalidTime', ...
              't_waypoints 必须严格递增。');
    end

    t_waypoints = t_waypoints(:).';

    % ============================================================
    % Step 1：生成 waypoint 速度
    % ============================================================
    qd_waypoints = zeros(n, M);

    switch lower(options.velocity_mode)

        case 'zero'
            % 全部设为 0
            qd_waypoints = zeros(n, M);

        case 'fd'
            % 使用中心差分估计中间点速度
            %
            % 边界：
            %   qd(:,1) = 0
            %   qd(:,M) = 0
            %
            % 中间：
            %   qd(:,k) = (q_{k+1} - q_{k-1}) / (t_{k+1} - t_{k-1})

            qd_waypoints(:,1) = zeros(n,1);
            qd_waypoints(:,M) = zeros(n,1);

            for k = 2:M-1
                qd_waypoints(:,k) = (q_waypoints(:,k+1) - q_waypoints(:,k-1)) ...
                                    / (t_waypoints(k+1) - t_waypoints(k-1));
            end

        case 'custom'
            if ~isfield(options, 'qd_waypoints')
                error('CreateMultiSegmentCubic:MissingVelocity', ...
                      '需要提供 options.qd_waypoints。');
            end

            qd_waypoints = options.qd_waypoints;

            if ~isequal(size(qd_waypoints), size(q_waypoints))
                error('CreateMultiSegmentCubic:VelocitySizeMismatch', ...
                      'qd_waypoints 尺寸必须与 q_waypoints 一致。');
            end

        otherwise
            error('CreateMultiSegmentCubic:UnknownMode', ...
                  '未知 velocity_mode。');
    end

    % ============================================================
    % Step 2：构造每一段 Cubic 参数
    % ============================================================
    segments = cell(1, M-1);

    for k = 1:M-1
        segments{k} = BuildCubicSegmentFromBoundary( ...
            q_waypoints(:,k), ...
            q_waypoints(:,k+1), ...
            qd_waypoints(:,k), ...
            qd_waypoints(:,k+1), ...
            t_waypoints(k), ...
            t_waypoints(k+1));
    end

    % ============================================================
    % 输出
    % ============================================================
    traj.q_waypoints = q_waypoints;
    traj.qd_waypoints = qd_waypoints;
    traj.t_waypoints = t_waypoints;
    traj.segments = segments;
    traj.type = 'multi_cubic';
end
