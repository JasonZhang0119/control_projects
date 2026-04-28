function profile = CreateTrapezoidalProfile(q0, qf, v0, vf, a_max, t_total)
%CREATETRAPEZOIDALPROFILE 创建带非零初末速度的梯形速度轨迹
%
% Parameters
% ----------
% q0 : double
%     初始位置。
%
% qf : double
%     终止位置。
%
% v0 : double
%     初始速度。
%
% vf : double
%     终止速度。
%
% a_max : double
%     最大加速度幅值，必须为正数。
%
% t_total : double
%     总运动时间，必须为正数。
%
% Returns
% -------
% profile : struct
%     梯形速度轨迹参数。

    if a_max <= 0
        error('CreateTrapezoidalProfile:InvalidAcceleration', ...
              'a_max 必须为正数。');
    end

    if t_total <= 0
        error('CreateTrapezoidalProfile:InvalidTotalTime', ...
              't_total 必须为正数。');
    end

    dq = qf - q0;

    if abs(dq) < 1e-12
        direction = 1;
    else
        direction = sign(dq);
    end

    s = abs(dq);
    a = abs(a_max);
    T = t_total;

    % 转换到正运动方向坐标
    u0 = direction * v0;
    uf = direction * vf;

    % 本实现假设轨迹整体沿 q0 -> qf 方向运动，
    % 且巡航速度 vc 不小于初末速度。
    A = a * T + u0 + uf;

    discriminant = A^2 - 2 * (u0^2 + uf^2 + 2 * a * s);

    if discriminant < -1e-12
        error('CreateTrapezoidalProfile:InfeasibleTrajectory', ...
              '给定 T、a_max、q0、qf、v0、vf 下不存在实数梯形轨迹。');
    end

    discriminant = max(discriminant, 0);

    vc = 0.5 * (A - sqrt(discriminant));

    if vc < max(u0, uf) - 1e-10
        error('CreateTrapezoidalProfile:InvalidCruiseVelocity', ...
              ['求得的巡航速度小于初末速度。该情况不满足标准“加速-匀速-减速”梯形假设，' ...
               '可能需要使用减速-匀速-加速、三角型，或更一般的带约束轨迹规划。']);
    end

    t_acc = (vc - u0) / a;
    t_dec = (vc - uf) / a;
    t_flat = T - t_acc - t_dec;

    if t_acc < -1e-10 || t_dec < -1e-10 || t_flat < -1e-10
        error('CreateTrapezoidalProfile:InvalidSegmentTime', ...
              '计算得到的某个时间段为负，轨迹不可行。');
    end

    t_acc = max(t_acc, 0);
    t_dec = max(t_dec, 0);
    t_flat = max(t_flat, 0);

    profile.q0 = q0;
    profile.qf = qf;
    profile.v0 = v0;
    profile.vf = vf;
    profile.direction = direction;
    profile.distance = s;
    profile.a_max = a;
    profile.v_cruise = vc;
    profile.t_acc = t_acc;
    profile.t_flat = t_flat;
    profile.t_dec = t_dec;
    profile.t_total = T;
    profile.t1 = t_acc;
    profile.t2 = t_acc + t_flat;
    profile.t3 = T;
end