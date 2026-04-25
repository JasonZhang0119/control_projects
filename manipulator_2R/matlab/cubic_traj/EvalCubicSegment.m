function [q, q_dot, q_ddot] = EvalCubicSegment(seg, t)
%EVALCUBICSEGMENT Evaluate a cubic segment at time t.

    t = min(max(t, seg.t0), seg.tf);
    tau = t - seg.t0;

    q = seg.a + seg.b * tau + seg.c * tau^2 + seg.d * tau^3;
    q_dot = seg.b + 2 * seg.c * tau + 3 * seg.d * tau^2;
    q_ddot = 2 * seg.c + 6 * seg.d * tau;
end
