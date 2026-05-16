function [q, q_dot, q_ddot] = QuinticInterp(q0, qf, q0_dot, qf_dot, q0_ddot, qf_ddot, t0, tf, t)
%QUINTICINTERP 五次多项式轨迹插补
%
% 参数
% ----------
% q0, qf : double, size (n, 1)
%     初始/终止位置。
%
% q0_dot, qf_dot : double, size (n, 1)
%     初始/终止速度。
%
% q0_ddot, qf_ddot : double, size (n, 1)
%     初始/终止加速度。
%
% t0, tf : double
%     初始/终止时间。
%
% t : double
%     当前时间。
%
% 返回
% -------
% q, q_dot, q_ddot : double, size (n, 1)
%     当前期望位置、速度、加速度。

    q0 = q0(:);
    qf = qf(:);
    q0_dot = q0_dot(:);
    qf_dot = qf_dot(:);
    q0_ddot = q0_ddot(:);
    qf_ddot = qf_ddot(:);

    n = numel(q0);

    if numel(qf) ~= n || numel(q0_dot) ~= n || numel(qf_dot) ~= n || ...
       numel(q0_ddot) ~= n || numel(qf_ddot) ~= n
        error('QuinticInterp:DimensionMismatch', ...
              '所有位置、速度、加速度向量维度必须一致。');
    end

    if tf <= t0
        error('QuinticInterp:InvalidTime', ...
              'tf 必须大于 t0。');
    end

    t = min(max(t, t0), tf);

    T = tf - t0;
    tau = t - t0;

    % 五次多项式：
    %   q(t) = a0 + a1*tau + a2*tau^2 + a3*tau^3 + a4*tau^4 + a5*tau^5
    %
    % 系数由六个边界条件确定：
    %   q(t0)      = q0
    %   q_dot(t0)  = q0_dot
    %   q_ddot(t0) = q0_ddot
    %   q(tf)      = qf
    %   q_dot(tf)  = qf_dot
    %   q_ddot(tf) = qf_ddot

    a0 = q0;
    a1 = q0_dot;
    a2 = 0.5 * q0_ddot;

    delta_q = qf - (a0 + a1 * T + a2 * T^2);
    delta_v = qf_dot - (a1 + 2 * a2 * T);
    delta_a = qf_ddot - (2 * a2);

    a3 = (10 * delta_q - 4 * delta_v * T + 0.5 * delta_a * T^2) / T^3;
    a4 = (-15 * delta_q + 7 * delta_v * T - delta_a * T^2) / T^4;
    a5 = (6 * delta_q - 3 * delta_v * T + 0.5 * delta_a * T^2) / T^5;

    q = a0 + a1 * tau + a2 * tau^2 + a3 * tau^3 + a4 * tau^4 + a5 * tau^5;

    q_dot = a1 + ...
            2 * a2 * tau + ...
            3 * a3 * tau^2 + ...
            4 * a4 * tau^3 + ...
            5 * a5 * tau^4;

    q_ddot = 2 * a2 + ...
             6 * a3 * tau + ...
             12 * a4 * tau^2 + ...
             20 * a5 * tau^3;
end