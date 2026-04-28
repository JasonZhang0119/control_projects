function [M, h, G] = Robot2R_Dynamics(q, qd, param)
%ROBOT2R_DYNAMICS 计算 2R 平面机械臂的简化动力学模型
%
% 本函数采用教材里常见的简化形式：
%
%   M(theta) * theta_dd + c(theta, theta_d) + g(theta) = tau
%
% 其中：
%   - M(theta)  : 惯性矩阵
%   - c(theta, theta_d) : 科里奥利/离心项
%   - g(theta)  : 重力项
%
% 为了兼容现有控制器接口，本函数仍返回：
%   [M, h, G]
% 其中：
%   h = c
%   G = g
%
% 输入
% ----
% q : double, size (2,1) or (1,2)
%     当前关节角 [q1; q2]
%
% qd : double, size (2,1) or (1,2)
%     当前关节角速度 [qd1; qd2]
%
% param : struct
%     至少应包含：
%         l1      : 第一根连杆长度
%         l2      : 第二根连杆长度
%         m1      : 第一根连杆质量
%         m2      : 第二根连杆质量
%
%     可选字段：
%         I1      : 第一根连杆绕质心的转动惯量
%         I2      : 第二根连杆绕质心的转动惯量
%         lc1     : 第一根连杆质心到关节 1 的距离
%         lc2     : 第二根连杆质心到关节 2 的距离
%         g_const : 重力加速度，默认 9.81
%
% 输出
% ----
% M : double, size (2,2)
%     惯性矩阵
%
% h : double, size (2,1)
%     非线性项中的速度相关部分，h = c
%
% G : double, size (2,1)
%     重力项

    q = q(:);
    qd = qd(:);

    if numel(q) ~= 2 || numel(qd) ~= 2
        error('Robot2R_Dynamics:DimensionMismatch', ...
              'q 和 qd 都必须是 2 维向量。');
    end

    % ------------------------------------------------------------
    % 读取参数
    % ------------------------------------------------------------
    l1 = get_param_value(param, {'l1', 'L1'});
    l2 = get_param_value(param, {'l2', 'L2'});
    m1 = get_param_value(param, {'m1'});
    m2 = get_param_value(param, {'m2'});

    g0 = get_param_value(param, {'g_const', 'g'}, 9.81);

    % 简化模型默认采用均匀杆，并将质心放在杆长中点
    lc1 = get_param_value(param, {'lc1'}, l1 / 2);
    lc2 = get_param_value(param, {'lc2'}, l2 / 2);

    % 若未提供转动惯量，则使用均匀细杆近似
    I1 = get_param_value(param, {'I1'}, (1 / 12) * m1 * l1^2);
    I2 = get_param_value(param, {'I2'}, (1 / 12) * m2 * l2^2);

    q1 = q(1);
    q2 = q(2);
    dq1 = qd(1);
    dq2 = qd(2);

    % ------------------------------------------------------------
    % 惯性矩阵 M(theta)
    % ------------------------------------------------------------
    c2 = cos(q2);

    M11 = I1 + I2 + m1 * lc1^2 + m2 * (l1^2 + lc2^2 + 2 * l1 * lc2 * c2);
    M12 = I2 + m2 * (lc2^2 + l1 * lc2 * c2);
    M22 = I2 + m2 * lc2^2;

    M = [M11, M12;
         M12, M22];

    % ------------------------------------------------------------
    % 科里奥利/离心项 c(theta, theta_d)
    % ------------------------------------------------------------
    s2 = sin(q2);
    h1 = -m2 * l1 * lc2 * s2 * (2 * dq1 * dq2 + dq2^2);
    h2 =  m2 * l1 * lc2 * s2 * dq1^2;

    % ------------------------------------------------------------
    % 重力项 g(theta)
    % ------------------------------------------------------------
    G1 = (m1 * lc1 + m2 * l1) * g0 * cos(q1) + m2 * lc2 * g0 * cos(q1 + q2);
    G2 = m2 * lc2 * g0 * cos(q1 + q2);

    G = [G1; G2];

    % ------------------------------------------------------------
    % 非线性项中的速度相关部分：h = c
    % 重力项单独返回为 G，便于控制器前馈补偿
    % ------------------------------------------------------------
    h = [h1; h2];
end


function value = get_param_value(param, names, default_value)
%GET_PARAM_VALUE 从 param 中按顺序查找字段，找不到则返回默认值

    if nargin < 3
        default_value = [];
    end

    value = default_value;

    for i = 1:numel(names)
        name = names{i};
        if isfield(param, name)
            value = param.(name);
            return;
        end
    end

    if isempty(value)
        error('Robot2R_Dynamics:MissingField', ...
              'param 中缺少必要字段：%s', strjoin(names, ' / '));
    end
end
