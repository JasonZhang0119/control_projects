function T_all = FK_DH(q, dh_table, joint_types, method)
%FK_DH 基于 DH 参数表的正向运动学计算（支持 R/P 关节）
%
% 描述
% -----
% 本函数根据给定的 DH 参数表和关节变量 q，计算串联机械臂从基座到各个关节坐标系的
% 齐次变换矩阵 T_0_i。
%
% 支持：
%   - Revolute（转动关节，'R'）
%   - Prismatic（伸缩关节，'P'）
%
% 参数
% ----------
% q : double, size (n, 1)
%     关节变量向量
%     - 对于 R 关节：q(i) 为关节角（单位：rad）
%     - 对于 P 关节：q(i) 为关节位移（单位：长度）
%
% dh_table : double, size (n, 4)
%     DH 参数表，每一行表示一个关节：
%
%         [a_i, alpha_i, d_i_offset, theta_i_offset]
%
%     各参数含义：
%         a_i             : 连杆长度（沿 x 轴）
%         alpha_i         : 连杆扭转角（绕 x 轴）
%         d_i_offset      : 沿 z 轴的固定偏移
%         theta_i_offset  : 关节角的固定偏移
%
%     注意：
%         实际参与计算的 DH 参数为：
%
%         - Revolute 关节：
%               theta_i = theta_i_offset + q(i)
%               d_i     = d_i_offset
%
%         - Prismatic 关节：
%               theta_i = theta_i_offset
%               d_i     = d_i_offset + q(i)
%
% joint_types : char 或 string, size (n, 1)
%     每个关节的类型：
%         'R' : 转动关节
%         'P' : 伸缩关节
%
% method : char 或 string（可选）
%     DH 建模方式：
%         'standard' : 标准 DH（默认）
%         'craig' 或 'craig' : 改进 DH（Craig）
%
% 返回
% -------
% T_all : double, size (4, 4, n + 1)
%     从基座坐标系到各关节坐标系的齐次变换矩阵集合：
%
%         T_all(:, :, 1)     : T_0_0 = 单位矩阵
%         T_all(:, :, i + 1) : T_0_i
%
%     末端位姿：
%         T_0_n = T_all(:, :, end)
%
% 说明
% -----
% 1. 本函数采用"右乘"累积变换：
%
%        T = T * A_i
%
%    表示从基座开始逐步叠加每一节连杆的变换。
%
% 2. 本实现将"结构参数（DH表）"与"运动变量（q）"解耦，
%    是通用机器人建模的标准方式。
%
% 3. 输出的 T_all 可直接用于：
%    - Jacobian 计算
%    - 末端位置提取
%    - 动力学建模
%

    if nargin < 4
        method = 'standard';
    end

    q = q(:);
    n = size(dh_table, 1);

    if numel(q) ~= n
        error('FK_DH:DimensionMismatch', ...
              'q 的长度必须与 dh_table 行数一致');
    end

    if numel(joint_types) ~= n
        error('FK_DH:JointTypeMismatch', ...
              'joint_types 长度必须与关节数一致');
    end

    T_all = zeros(4, 4, n + 1);
    T_all(:, :, 1) = eye(4);

    T = eye(4);

    for i = 1:n
        alpha = dh_table(i, 1);
        a = dh_table(i, 2);
        d_offset = dh_table(i, 3);
        theta_offset = dh_table(i, 4);

        joint_type = upper(joint_types(i));

        if joint_type == 'R'
            theta = theta_offset + q(i);
            d = d_offset;

        elseif joint_type == 'P'
            theta = theta_offset;
            d = d_offset + q(i);

        else
            error('FK_DH:InvalidJointType', ...
                  '关节类型必须为 ''R'' 或 ''P''');
        end

        A = DHTransform(a, alpha, d, theta, method);

        T = T * A;

        T_all(:, :, i + 1) = T;
    end
end