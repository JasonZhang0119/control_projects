function [q, q_dot, q_ddot] = EvalMultiSegmentCubic(traj, t)
%EVALMULTISEGMENTCUBIC Evaluate a multi-segment cubic trajectory.

    segments = traj.segments;
    t_waypoints = traj.t_waypoints;

    if t <= t_waypoints(1)
        seg = segments{1};
    elseif t >= t_waypoints(end)
        seg = segments{end};
    else
        k = find(t >= t_waypoints(1:end-1) & t < t_waypoints(2:end), 1, 'first');
        seg = segments{k};
    end

    [q, q_dot, q_ddot] = EvalCubicSegment(seg, t);
end
