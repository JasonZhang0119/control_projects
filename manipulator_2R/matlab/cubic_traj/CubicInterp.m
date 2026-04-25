function [q, q_dot, q_ddot] = CubicInterp(q0, qf, q0_dot, qf_dot, t0, tf, t)
%CUBICINTERP 三次多项式轨迹插补
%
% 参数
% ----------
% q0, qf : double, size (n, 1)
%     初始/终止位置。
%
% q0_dot, qf_dot : double, size (n, 1)
%     初始/终止速度。
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

    n = numel(q0);

    if numel(qf) ~= n || numel(q0_dot) ~= n || numel(qf_dot) ~= n
        error('CubicInterp:DimensionMismatch', ...
              'q0、qf、q0_dot、qf_dot 的维度必须一致。');
    end

    if tf <= t0
        error('CubicInterp:InvalidTime', ...
              'tf 必须大于 t0。');
    end

    t = min(max(t, t0), tf);

    T = tf - t0;
    tau = t - t0;

    % 三次多项式：
    % q(t) = a0 + a1*tau + a2*tau^2 + a3*tau^3
    %

    a0 = q0;
    a1 = q0_dot;
    a2 = (3 * qf - 3 * q0 - 2 * q0_dot * T - qf_dot * T) / T^2;
    a3 = (2 * q0 + (q0_dot + qf_dot) * T - 2 * qf) / T^3;

    q = a0 + a1 * tau + a2 * tau^2 + a3 * tau^3;
    q_dot = a1+ 2 * a2 * tau + 3 * a3 * tau^2;
    q_ddot = 2 * a2 + 6 * a3 * tau;
end