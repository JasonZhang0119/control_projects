function profile = CreateTrapezoidalProfile(q0, qf, a_max, t_total)
%CREATETRAPEZOIDALPROFILE 基于给定总时间和加速度约束创建梯形速度轨迹
%
% 参数
% ----------
% q0 : double
%     初始位置。
%
% qf : double
%     终止位置。
%
% a_max : double
%     最大加速度，必须为正数。
%
% t_total : double
%     规定总运动时间，必须为正数。
%
% 返回
% -------
% profile : struct
%     梯形/三角速度轨迹参数结构体。

    if a_max <= 0
        error('CreateTrapezoidalProfile:InvalidAcceleration', ...
              'a_max 必须为正数。');
    end

    if t_total <= 0
        error('CreateTrapezoidalProfile:InvalidTotalTime', ...
              't_total 必须为正数。');
    end

    dq = qf - q0;
    distance = abs(dq);

    if distance < 1e-12
        profile.q0 = q0;
        profile.qf = qf;
        profile.direction = 1;
        profile.distance = 0;
        profile.a_max = abs(a_max);
        profile.t_acc = 0;
        profile.t_flat = t_total;
        profile.t_dec = 0;
        profile.t_total = t_total;
        profile.v_peak = 0;
        profile.is_triangle = false;
        profile.a_min = 0;
        return;
    end

    direction = sign(dq);
    a = abs(a_max);
    T = t_total;

    % ============================================================
    % Step 1：可行性检查
    % ============================================================
    % 在给定总时间 T 内，若采用最快的对称三角速度曲线：
    %
    %     distance = a_min * (T/2)^2
    %
    % 因此：
    %
    %     a_min = 4 * distance / T^2
    %
    % 若 a < a_min，则说明即使全程加速再减速，也无法在规定时间内到达。

    a_min = 4 * distance / T^2;

    if a < a_min - 1e-12
        error('CreateTrapezoidalProfile:InfeasibleAcceleration', ...
              ['给定加速度过小，无法在规定时间内完成轨迹。' ...
               ' 最小所需加速度为 %.6g，当前 a_max 为 %.6g。'], ...
               a_min, a);
    end

    % ============================================================
    % Step 2：求解加速段时间 t_b
    % ============================================================
    % 对称加减速下：
    %
    %     distance = a * t_b * (T - t_b)
    %
    % 等价于：
    %
    %     t_b^2 - T*t_b + distance/a = 0
    %
    % 解为：
    %
    %     t_b = (T - sqrt(T^2 - 4*distance/a)) / 2
    %
    % 选择较小根，保证 t_b <= T/2。

    discriminant = T^2 - 4 * distance / a;

    if discriminant < -1e-12
        error('CreateTrapezoidalProfile:InvalidDiscriminant', ...
              '判别式为负，无法生成实数轨迹。');
    end

    discriminant = max(discriminant, 0);

    t_b = 0.5 * (T - sqrt(discriminant));

    % ============================================================
    % Step 3：计算轨迹参数
    % ============================================================

    t_acc = t_b;
    t_dec = t_b;
    t_flat = T - 2 * t_b;
    v_peak = a * t_b;

    if abs(t_flat) < 1e-12
        t_flat = 0;
    end

    is_triangle = (t_flat == 0);

    profile.q0 = q0;
    profile.qf = qf;
    profile.direction = direction;
    profile.distance = distance;
    profile.a_max = a;
    profile.t_acc = t_acc;
    profile.t_flat = t_flat;
    profile.t_dec = t_dec;
    profile.t_total = T;
    profile.v_peak = v_peak;
    profile.is_triangle = is_triangle;
    profile.a_min = a_min;
end