function J = Jacobian2R_Analytic(q, param)
%JACOBIAN2R_ANALYTIC 平面 2R 机械臂解析 Jacobian
%
% 参数
% ----------
% q : double, size (2, 1)
%     关节角向量：
%         q(1) : 第 1 个关节角，单位 rad
%         q(2) : 第 2 个关节角，单位 rad
%
% param : struct
%     2R 机械臂参数结构体，至少包含：
%         l1 : 第 1 根连杆长度
%         l2 : 第 2 根连杆长度
%
% 返回
% -------
% J : double, size (2, 2)
%     末端位置关于关节角的解析 Jacobian。
%
%     满足：
%         p_dot = J(q) * q_dot
%
%     其中：
%         p = [x; y]
%         q = [q1; q2]

    q = q(:);

    if numel(q) ~= 2
        error('Jacobian2R_Analytic:DimensionMismatch', ...
              'q 必须为长度为 2 的关节角向量。');
    end

    q1 = q(1);
    q2 = q(2);

    l1 = param.l1;
    l2 = param.l2;

    J = zeros(2, 2);

    % TODO:
    % 根据：
    %   x = l1*cos(q1) + l2*cos(q1 + q2)
    %   y = l1*sin(q1) + l2*sin(q1 + q2)
    %
    % 填写：
    %   J(1,1) = dx / dq1
    %   J(1,2) = dx / dq2
    %   J(2,1) = dy / dq1
    %   J(2,2) = dy / dq2

    J(1, 1) = -l1 * sin(q1) - l2 * sin(q1 + q2) ;
    J(1, 2) = -l2 * sin(q1 + q2) ;

    J(2, 1) = l1 * cos(q1) + l2 * cos(q1 + q2);
    J(2, 2) = l2 * cos(q1 + q2);
end

