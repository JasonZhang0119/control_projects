function [q_sol, info] = IKPosition_Numerical(p_d, q0, param, options)
%IKPOSITION_NUMERICAL 使用位置误差求解数值逆运动学
%
% Parameters
% ----------
%
%
% p_d : double, size (m, 1)
%     目标末端位置。
%
% q0 : double, size (n, 1)
%     初始关节角。
%
% options : struct, optional
%     数值 IK 配置参数。
%
% Returns
% -------
% q_sol : double, size (n, 1)
%     求解得到的关节角。
%
% info : struct
%     求解过程信息。
%
% Notes
% -----
% - 该函数适合只关心末端位置的任务。
% - 对 2R 平面机械臂，p_d 通常为 size (2, 1)。
% - 对空间机械臂位置控制，p_d 通常为 size (3, 1)。

    if nargin < 4
        options = struct();
    end

    max_iter = get_option(options, 'max_iter', 100);
    tol      = get_option(options, 'tol', 1e-6);
    alpha    = get_option(options, 'alpha', 1.0);

    q = q0(:);
    p_d = p_d(:);

    n = numel(q);
    m = numel(p_d);

    err_norm_hist = zeros(max_iter, 1);
    converged = false;

    for iter = 1:max_iter

        % ================================================================
        % step 1: 求解f(q)
        % ================================================================
        T_all = FK_DH(q, param.dh_table, param.joint_types, param.dh_method);
        T_0_tool = T_all(:, :, end) * param.T_tool;
        p = T_0_tool (1:m, 4);

        if numel(p) ~= m
            error('IKPosition_Numerical:InvalidPositionSize', ...
                  'fk_pos_fun 返回的位置维度必须与 p_d 一致。');
        end

        p = p(:);

        % ================================================================
        % step 2: 构造位置误差。
        % ================================================================
        e = p_d - p;

        err_norm = norm(e);
        err_norm_hist(iter) = err_norm;

        if err_norm < tol
            converged = true;
            break;
        end

        % ================================================================
        % step 3: 调用你自己的位置 Jacobian 函数。
        % ================================================================
        Jp_full = JacobianFromTAll(T_all, param.joint_types, param.T_tool);
        Jv = Jp_full(4:6, :);
        Jp = Jv(1:m, :);

        if size(Jp, 1) ~= m || size(Jp, 2) ~= n
            error('IKPosition_Numerical:InvalidJacobianSize', ...
                  '位置 Jacobian 尺寸必须为 m x n。');
        end

        % ================================================================
        % step 4: Newton-Type Method
        % ================================================================
        delta_q = pinv(Jp) * e;
        q = q + alpha * delta_q;
    end

    q_sol = q;

    info = struct();
    info.converged = converged;
    info.iter = iter;
    info.final_error_norm = err_norm_hist(iter);
    info.err_norm_hist = err_norm_hist(1:iter);
end


function value = get_option(options, field_name, default_value)
%GET_OPTION 从 options 中读取字段，若不存在则返回默认值

    if isfield(options, field_name)
        value = options.(field_name);
    else
        value = default_value;
    end
end