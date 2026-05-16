function seg = BuildCubicSegmentFromBoundary(q0, qf, q0_dot, qf_dot, t0, tf)
%BUILDCUBICSEGMENTFROMBOUNDARY Build cubic segment from endpoint states.

    q0 = q0(:);
    qf = qf(:);
    q0_dot = q0_dot(:);
    qf_dot = qf_dot(:);

    if numel(q0) ~= numel(qf) || numel(q0) ~= numel(q0_dot) || numel(q0) ~= numel(qf_dot)
        error('BuildCubicSegmentFromBoundary:DimensionMismatch', ...
              'All boundary vectors must have the same size.');
    end

    if tf <= t0
        error('BuildCubicSegmentFromBoundary:InvalidTime', ...
              'tf must be greater than t0.');
    end

    T = tf - t0;

    seg.t0 = t0;
    seg.tf = tf;
    seg.h = T;
    seg.q0 = q0;
    seg.qf = qf;
    seg.q0_dot = q0_dot;
    seg.qf_dot = qf_dot;

    seg.a = q0;
    seg.b = q0_dot;
    seg.c = (3 * qf - 3 * q0 - 2 * q0_dot * T - qf_dot * T) / T^2;
    seg.d = (2 * q0 + (q0_dot + qf_dot) * T - 2 * qf) / T^3;
end
