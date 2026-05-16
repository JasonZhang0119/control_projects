clear; clc;

param = Param2R('standard');

for i = 1:100
    q = (rand(2,1) - 0.5) * 2*pi;

    p1 = FK2R_Analytic(q, param);

    T_all = FK_DH(q, param.dh_table, param.joint_types, param.dh_method);
    T02 = T_all(:, :, end);
    p2 = T02(1:2, 4);

    err = norm(p1 - p2);

    if err > 1e-10
        disp('q = ');
        disp(q);

        disp('p1 analytic = ');
        disp(p1);

        disp('p2 DH = ');
        disp(p2);

        disp('error = ');
        disp(err);

        error('main_kinematics:FKMismatch', ...
              'FK mismatch: %.3e', err);
    end
end

disp('FK check passed!');


%%
eps_fd = 1e-6;
tol = 1e-6;

for i = 1:100
    q = (rand(2, 1) - 0.5) * 2 * pi;

    J_analytic = Jacobian2R_Analytic(q, param);

    J_fd = zeros(2, 2);

    for j = 1:2
        dq = zeros(2, 1);
        dq(j) = eps_fd;

        p_plus = FK2R_Analytic(q + dq, param);
        p_minus = FK2R_Analytic(q - dq, param);

        J_fd(:, j) = (p_plus - p_minus) / (2 * eps_fd);
    end

    err = norm(J_analytic - J_fd);

    if err > tol
        disp('q = ');
        disp(q);

        disp('J_analytic = ');
        disp(J_analytic);

        disp('J_fd = ');
        disp(J_fd);

        disp('error = ');
        disp(err);

        error('main_jacobian_check:JacobianMismatch', ...
              'Jacobian mismatch: %.3e', err);
    end
end

disp('Jacobian analytic check passed!');

%%

tol = 1e-10;

for i = 1:100
    q = (rand(2, 1) - 0.5) * 2 * pi;

    % ===== 解析 Jacobian =====
    J_analytic = Jacobian2R_Analytic(q, param);

    % ===== 基于 DH 变换链的通用 Jacobian =====
    T_all = FK_DH(q, param.dh_table, param.joint_types, param.dh_method);

    if isfield(param, 'T_tool')
        T_tool = param.T_tool;
    else
        T_tool = eye(4);
    end

    J_geo = JacobianFromTAll(T_all, param.joint_types, T_tool);

    % 平面 2R 只比较末端 x-y 线速度部分
    J_tall_xy = J_geo(4:5, :);

    err = norm(J_analytic - J_tall_xy);

    if err > tol
        disp('q = ');
        disp(q);

        disp('J_analytic = ');
        disp(J_analytic);

        disp('J_tall_xy = ');
        disp(J_tall_xy);

        disp('J_geo = ');
        disp(J_geo);

        disp('error = ');
        disp(err);

        error('main_jacobian_tall_check:JacobianMismatch', ...
              'Jacobian mismatch: %.3e', err);
    end
end

disp('Jacobian analytic vs T_all check passed!');