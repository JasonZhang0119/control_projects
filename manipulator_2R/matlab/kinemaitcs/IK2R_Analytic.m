function q_all = IK2R_Analytic(p, param)
%IK2R_ANALYTIC Analytic inverse kinematics for a planar 2R robot.
%
% Parameters
% ----------
% p : double, size (2,1) or (1,2)
%     End-effector position [x; y].
%
% param : struct
%     Robot parameter struct with fields:
%         l1, l2
%
% Returns
% -------
% q_all : double, size (2,2)
%     Two analytic IK solutions.
%     q_all(:,1) : elbow-up branch
%     q_all(:,2) : elbow-down branch

    p = p(:);

    if numel(p) ~= 2
        error('IK2R_Analytic:DimensionMismatch', ...
              'p must be a 2-element position vector.');
    end

    x = p(1);
    y = p(2);

    l1 = param.l1;
    l2 = param.l2;

    r2 = x^2 + y^2;
    c2 = (r2 - l1^2 - l2^2) / (2 * l1 * l2);

    % Guard against tiny numerical overshoot.
    if abs(c2) > 1 + 1e-12
        error('IK2R_Analytic:UnreachableTarget', ...
              'Target point is outside the reachable workspace.');
    end

    c2 = min(max(c2, -1), 1);

    s2_pos = sqrt(max(0, 1 - c2^2));
    s2_neg = -s2_pos;

    q2_up = atan2(s2_pos, c2);
    q2_down = atan2(s2_neg, c2);

    q1_up = atan2(y, x) - atan2(l2 * sin(q2_up), l1 + l2 * cos(q2_up));
    q1_down = atan2(y, x) - atan2(l2 * sin(q2_down), l1 + l2 * cos(q2_down));

    q_all = [
        wrapToPiLocal(q1_up),   wrapToPiLocal(q1_down);
        wrapToPiLocal(q2_up),   wrapToPiLocal(q2_down)
    ];
end

function a = wrapToPiLocal(a)
%WRAPTOPILOCAL Wrap angle to [-pi, pi].
    a = mod(a + pi, 2*pi) - pi;
end
