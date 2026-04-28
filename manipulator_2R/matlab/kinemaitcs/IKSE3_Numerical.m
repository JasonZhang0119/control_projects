function [q_sol, info] = IKSE3_Numerical(T_sd, q0, param, options)
%IKSE3_NUMERICAL 使用 SE(3) 位姿误差求解数值逆运动学。
%
% 输入
% ----
% T_sd : double, size (4, 4)
%     目标末端位姿，属于 SE(3)。
%
% q0 : double, size (n, 1)
%     初始关节角。
%
% param : struct
%     机器人参数结构体，函数内部会调用：
%         - FK_DH
%         - JacobianFromTAll
%         - AdjointSE3
%
% options : struct, optional
%     数值 IK 配置参数。
%     常用字段包括：
%         max_iter : 最大迭代次数
%         tol_pos  : 位置误差容差
%         tol_rot  : 姿态误差容差
%         alpha    : 步长系数
%         lambda   : DLS 阻尼系数
%         frame    : 'body' 或 'space'
%
% 输出
% ----
% q_sol : double, size (n, 1)
%     求解得到的关节角。
%
% info : struct
%     求解过程信息，包括是否收敛、迭代次数和误差历史。
%
% 说明
% ----
% - 本函数使用迭代型数值 IK。
% - 默认使用 damped least squares。
% - 若 options.frame = 'space'，则误差 twist 与 Jacobian 也应采用 space frame。
% - 若 options.frame = 'body'，则误差 twist 与 Jacobian 也应采用 body frame。

    if nargin < 4 || isempty(options)
        options = struct();
    end

    max_iter = get_option(options, 'max_iter', 100);
    tol_pos  = get_option(options, 'tol_pos', 1e-6);
    tol_rot  = get_option(options, 'tol_rot', 1e-6);
    alpha    = get_option(options, 'alpha', 1.0);
    lambda   = get_option(options, 'lambda', 1e-3);
    frame    = lower(get_option(options, 'frame', 'body'));

    q = q0(:);
    n = numel(q);

    err_norm_hist = zeros(max_iter, 1);
    pos_err_hist  = zeros(max_iter, 1);
    rot_err_hist  = zeros(max_iter, 1);

    converged = false;
    iter = 0;

    for k = 1:max_iter
        iter = k;

        % ============================================================
        % 计算当前末端位姿 T_sb(q)
        % ============================================================
        T_all = FK_DH(q, param.dh_table, param.joint_types, param.dh_method);
        T_sb = T_all(:, :, end) * param.T_tool;

        % ============================================================
        % 计算 SE(3) 位姿误差 twist
        % ============================================================
        switch frame
            case 'body'
                T_err = inv(T_sb) * T_sd;
                V_err = SE3LogToVec(T_err);

            case 'space'
                T_err = T_sd * inv(T_sb);
                V_err = SE3LogToVec(T_err);

            otherwise
                error('IKSE3_Numerical:InvalidFrame', ...
                      'options.frame 必须为 ''body'' 或 ''space''。');
        end

        omega_err = V_err(1:3);
        v_err = V_err(4:6);

        rot_err = norm(omega_err);
        pos_err = norm(v_err);

        rot_err_hist(k) = rot_err;
        pos_err_hist(k) = pos_err;
        err_norm_hist(k) = norm(V_err);

        if pos_err < tol_pos && rot_err < tol_rot
            converged = true;
            break;
        end

        % ============================================================
        % 计算 Jacobian
        % ============================================================
        Js = JacobianFromTAll(T_all, param.joint_types, param.T_tool);

        switch frame
            case 'space'
                J = Js;
            case 'body'
                J = AdjointSE3(inv(T_sb)) * Js;
        end

        if size(J, 1) ~= 6 || size(J, 2) ~= n
            error('IKSE3_Numerical:InvalidJacobianSize', ...
                  'Jacobian 尺寸必须为 6 x n。');
        end

        % ============================================================
        % Damped Least Squares 更新
        % ============================================================
        % delta_q = J' / (J * J' + lambda^2 * eye(6)) * V_err;
        delta_q = pinv(J)* V_err;
        q = q + alpha * delta_q;
    end

    q_sol = q;

    info = struct();
    info.converged = converged;
    info.iter = iter;
    info.final_error_norm = err_norm_hist(iter);
    info.final_pos_error = pos_err_hist(iter);
    info.final_rot_error = rot_err_hist(iter);
    info.err_norm_hist = err_norm_hist(1:iter);
    info.pos_err_hist = pos_err_hist(1:iter);
    info.rot_err_hist = rot_err_hist(1:iter);
end


function value = get_option(options, field_name, default_value)
%GET_OPTION 从 options 中读取字段，若不存在则返回默认值。
    if isfield(options, field_name)
        value = options.(field_name);
    else
        value = default_value;
    end
end
