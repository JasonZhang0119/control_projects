function [q, info] = IK2R_Numerical(p, param, options)
%IK2R_NUMERICAL 平面 2R 机械臂数值逆运动学模板。
%
% 这是一个用于尝试数值逆解方法的骨架文件，例如：
%   - 牛顿迭代法
%   - Jacobian 伪逆法
%   - Jacobian 转置法
%   - 阻尼最小二乘法
%
% 输入
% ----
% p : double, size (2,1) or (1,2)
%     末端目标位置 [x; y]。
%
% param : struct
%     机器人参数结构体，期望包含：
%         l1, l2
%
% options : struct, optional
%     数值求解配置：
%         q0          : 初值，大小为 (2,1)
%         max_iter    : 最大迭代次数
%         tol         : 位置误差容差
%         step_size   : 步长 / 增益
%         method      : 'pinv', 'transpose', or 'dls'
%         lambda      : DLS 阻尼系数
%
% 输出
% ----
% q : double, size (2,1)
%     求得的关节解。
%
% info : struct
%     求解过程信息：
%         success
%         iterations
%         final_error
%         method

    if nargin < 3
        options = struct();
    end

    p = p(:);

    if numel(p) ~= 2
        error('IK2R_Numerical:DimensionMismatch', ...
              'p must be a 2-element position vector.');
    end

    if ~isfield(options, 'q0')
        options.q0 = [0; 0];
    end

    if ~isfield(options, 'max_iter')
        options.max_iter = 100;
    end

    if ~isfield(options, 'tol')
        options.tol = 1e-8;
    end

    if ~isfield(options, 'step_size')
        options.step_size = 1.0;
    end

    if ~isfield(options, 'method')
        options.method = 'pinv';
    end

    if ~isfield(options, 'lambda')
        options.lambda = 1e-3;
    end

    q = options.q0(:);

    if numel(q) ~= 2
        error('IK2R_Numerical:InitialGuessMismatch', ...
              'options.q0 must be a 2-element joint vector.');
    end

    info.success = false;
    info.iterations = 0;
    info.final_error = NaN;
    info.method = options.method;

    % ============================================================
    % 迭代求解主循环
    % ============================================================
    for iter = 1:options.max_iter
        p_hat = FK2R_Analytic(q, param);
        e = p - p_hat;

        err_norm = norm(e);
        info.iterations = iter;
        info.final_error = err_norm;

        if err_norm < options.tol
            info.success = true;
            break;
        end

        J = Jacobian2R_Analytic(q, param);

        % --------------------------------------------------------
        % TODO：选择你想研究的更新律
        %
        % 常见选项：
        %   1）牛顿法 / 伪逆法：
        %        dq = J \ e
        %        dq = pinv(J) * e
        %
        %   2）Jacobian 转置法：
        %        dq = J' * e
        %
        %   3）阻尼最小二乘法：
        %        dq = J' * inv(J*J' + lambda^2*I) * e
        %
        % 然后更新：
        %        q = q + step_size * dq
        % --------------------------------------------------------

        dq = zeros(2, 1);  % TODO: 替换为你选定的更新公式

        % TODO：可选地在每一步后对关节角做 wrap
        % q = wrapToPiLocal(q + options.step_size * dq);

        q = q + options.step_size * dq;
    end

    % ============================================================
    % 可选后处理
    % ============================================================
    % TODO:
    %   你可以选择对 q 做限幅、角度 wrap，或者重新计算末端位置，
    %   以便输出更详细的残差信息。

    if ~info.success
        info.final_error = norm(p - FK2R_Analytic(q, param));
    end
end

% -------------------------------------------------------------------------
% 局部函数：将角度包裹到 [-pi, pi]
% -------------------------------------------------------------------------
function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
