function seg = BuildCubicSegmentFromSpline(qk, qk1, mk, mk1, t0, tf)
%BUILDCUBICSEGMENTFROMSPLINE Build cubic spline segment from waypoint accelerations.

    qk = qk(:);
    qk1 = qk1(:);
    mk = mk(:);
    mk1 = mk1(:);

    if numel(qk) ~= numel(qk1) || numel(qk) ~= numel(mk) || numel(qk) ~= numel(mk1)
        error('BuildCubicSegmentFromSpline:DimensionMismatch', ...
              'All waypoint vectors must have the same size.');
    end

    if tf <= t0
        error('BuildCubicSegmentFromSpline:InvalidTime', ...
              'tf must be greater than t0.');
    end

    h = tf - t0;

    seg.t0 = t0;
    seg.tf = tf;
    seg.h = h;
    seg.q0 = qk;
    seg.qf = qk1;
    seg.m0 = mk;
    seg.mf = mk1;

    seg.a = qk;
    seg.b = (qk1 - qk) / h - h / 6 * (2 * mk + mk1);
    seg.c = mk / 2;
    seg.d = (mk1 - mk) / (6 * h);
end
