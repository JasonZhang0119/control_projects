function p = FK2R_Analytic(q, param)
%FK2R Forward kinematics of a planar 2R robot.
%
% Parameters
% ----------
% q : double, size (2,1)
% param : struct
%
% Returns
% -------
% p : double, size (2,1)

    q = q(:);

    q1 = q(1);
    q2 = q(2);

    l1 = param.l1;
    l2 = param.l2;

    x = l1 * cos(q1) + l2 * cos(q1 + q2);
    y = l1 * sin(q1) + l2 * sin(q1 + q2);

    p = [x; y];
end