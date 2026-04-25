function traj = CreateCubicSpline(q_waypoints, t_waypoints)
%CREATECUBICSPLINE Create a natural cubic spline trajectory.

    [n, M] = size(q_waypoints);

    if M < 2
        error('CreateCubicSpline:InvalidWaypointNumber', 'At least two waypoints are required.');
    end

    if numel(t_waypoints) ~= M
        error('CreateCubicSpline:TimeDimensionMismatch', 'The length of t_waypoints must match the number of waypoints.');
    end

    t_waypoints = t_waypoints(:).';

    if any(diff(t_waypoints) <= 0)
        error('CreateCubicSpline:InvalidTimeWaypoints', 't_waypoints must be strictly increasing.');
    end

    m_waypoints = zeros(n, M);
    A = zeros(M, M);
    A(1, 1) = 1;
    A(M, M) = 1;

    for k = 2:M-1
        hkm1 = t_waypoints(k) - t_waypoints(k-1);
        hk = t_waypoints(k + 1) - t_waypoints(k);
        A(k, k-1) = hkm1;
        A(k, k) = 2 * (hkm1 + hk);
        A(k, k+1) = hk;
    end

    for dim = 1:n
        rhs = zeros(M, 1);
        rhs(1) = 0;
        rhs(M) = 0;

        for k = 2:M-1
            qk = q_waypoints(dim, k);
            qk1 = q_waypoints(dim, k+1);
            qkm1 = q_waypoints(dim, k-1);
            tk = t_waypoints(k);
            tk1 = t_waypoints(k+1);
            tkm1 = t_waypoints(k-1);
            rhs(k) = 6 * ((qk1 - qk) / (tk1 - tk) - (qk - qkm1) / (tk - tkm1));
        end

        m_waypoints(dim, :) = (A \ rhs).';
    end

    segments = cell(1, M-1);
    for k = 1:M-1
        segments{k} = BuildCubicSegmentFromSpline( ...
            q_waypoints(:, k), ...
            q_waypoints(:, k+1), ...
            m_waypoints(:, k), ...
            m_waypoints(:, k+1), ...
            t_waypoints(k), ...
            t_waypoints(k+1));
    end

    traj.q_waypoints = q_waypoints;
    traj.t_waypoints = t_waypoints;
    traj.m_waypoints = m_waypoints;
    traj.segments = segments;
    traj.type = 'natural_cubic_spline';
end
