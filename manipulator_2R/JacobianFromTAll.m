function J = JacobianFromTAll(T_all, joint_types, T_tool)
%JACOBIANFROMTALL 基于正运动学变换链计算空间几何 Jacobian
%
% 参数
% ----------
% T_all : double, size (4, 4, n + 1)
%     从基座坐标系到各关节坐标系的齐次变换矩阵。
%
% joint_types : char 或 string, size (n, 1)
%     关节类型：
%         'R' : 转动关节
%         'P' : 移动关节
%
% T_tool : double, size (4, 4), 可选
%     末端工具坐标系相对于最后一个关节坐标系的固定变换。
%
% 返回
% -------
% J : double, size (6, n)
%     空间几何 Jacobian。
%     前 3 行为角速度 Jacobian Jw。
%     后 3 行为线速度 Jacobian Jv。

    if nargin < 3
        T_tool = eye(4);
    end

    n = size(T_all, 3) - 1;

    if numel(joint_types) ~= n
        error('JacobianFromTAll:JointTypeMismatch', ...
              'joint_types 长度必须与关节数一致。');
    end

    T_0_n = T_all(:, :, end);
    T_0_tool = T_0_n * T_tool;

    p_tool = T_0_tool(1:3, 4);

    J = zeros(6, n);

    for i = 1:n
        T_0_im1 = T_all(:, :, i);

        p_i = T_0_im1(1:3, 4);
        z_i = T_0_im1(1:3, 3);

        joint_type = upper(joint_types(i));

        if joint_type == 'R'
            % TODO:
            J(1:3, i) = z_i;
            J(4:6, i) = cross(z_i, (p_tool - p_i));

        elseif joint_type == 'P'
            % TODO:
            J(1:3, i) = zeros(3, 1);
            J(4:6, i) = z_i;

        else
            error('JacobianFromTAll:InvalidJointType', ...
                  '关节类型必须为 ''R'' 或 ''P''。');
        end
    end
end