function traj = CreateMultiSegmentTrapezoidal(q_waypoints, t_waypoints, options)
%CREATEMULTISEGMENTTRAPEZOIDAL Template for multi-segment trapezoidal motion planning.
%
% This function is a skeleton for a piecewise trapezoidal trajectory
% generator. The current file keeps the structure and leaves the core
% timing/profile construction as TODOs for later implementation.
%
% Inputs
% ------
% q_waypoints : double, size (n, M)
%     Waypoint matrix. Each column is one waypoint.
%
% t_waypoints : double, size (1, M) or (M, 1)
%     Strictly increasing waypoint times.
%
% options : struct, optional
%     Suggested fields:
%         a_max            : maximum acceleration
%         stop_at_waypoints : logical, default true
%         velocity_mode     : 'zero' | 'custom' | 'fd'
%
% Output
% ------
% traj : struct
%     Trajectory container for later evaluation.
%     Suggested fields:
%         type
%         q_waypoints
%         t_waypoints
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
    % Step 1: Prepare waypoint velocities
    % ============================================================
    %
    % TODO:
    %   Decide how to set waypoint velocities.
    %   For the first version, the recommended choice is:
    %       qd_waypoints = zeros(n, M)
    %
    %   Later you can extend this to:
    %       - finite-difference velocity estimation
    %       - custom waypoint velocities
    %       - continuous-through-waypoint profiles
    %
    qd_waypoints = zeros(n, M); %#ok<NASGU>

    % ============================================================
    % Step 2: Build per-segment profiles
    % ============================================================
    %
    % TODO:
    %   For each segment k:
    %       q0 = q_waypoints(:, k)
    %       qf = q_waypoints(:, k+1)
    %       t0 = t_waypoints(k)
    %       tf = t_waypoints(k+1)
    %
    %   Then construct a trapezoidal profile or a per-segment structure
    %   that can later be evaluated by EvalTrapezoidSegment.
    %
    segments = cell(1, M-1);

    for k = 1:M-1
        seg = struct();
        seg.t0 = t_waypoints(k);
        seg.tf = t_waypoints(k+1);
        seg.h = seg.tf - seg.t0;
        seg.q0 = q_waypoints(:, k);
        seg.qf = q_waypoints(:, k+1);
        seg.qd0 = qd_waypoints(:, k);   %#ok<NASGU>
        seg.qdf = qd_waypoints(:, k+1);  %#ok<NASGU>

        % TODO:
        %   Replace the placeholder with your trapezoidal profile builder.
        %   Typical choices:
        %       seg.profile = CreateTrapezoidalProfile(...)
        %       seg.profile = ...
        %
        seg.profile = []; %#ok<NASGU>

        segments{k} = seg;
    end

    % ============================================================
    % Output
    % ============================================================
    traj.type = 'multi_trapezoidal';
    traj.q_waypoints = q_waypoints;
    traj.t_waypoints = t_waypoints;
    traj.qd_waypoints = qd_waypoints;
    traj.segments = segments;
    traj.options = options;
end
