function param = Param2R(method)
%PARAM2R 创建平面 2R 机械臂参数结构体（含 DH 表）
%
% 参数
% ----------
% method : char 或 string
%     DH 建模方式：
%         'standard'  : 标准 DH
%         'modified'  : 改进 DH（Craig）
%
% 返回
% -------
% param : struct
%     机械臂参数结构体，包含：
%
%     几何参数：
%         l1, l2 : 连杆长度
%
%     质量参数（后续动力学用）：
%         m1, m2 : 连杆质量
%
%     运动学建模：
%         dh_table    : DH 参数表 (n x 4)
%         joint_types : 关节类型 (n x 1)
%         dh_method   : 使用的 DH 类型

    if nargin < 1
        method = 'standard';
    end

    % ===== 几何参数 =====
    param.l1 = 1.0;
    param.l2 = 0.8;

    % ===== 动力学参数（先占位）=====
    param.m1 = 1.0;
    param.m2 = 1.0;

    % ===== 关节类型（2R）=====
    param.joint_types = ['R'; 'R'];

    % ===== DH 表 =====
    % 每一行：[alpha, a, d_offset, theta_offset]

    switch lower(method)

        case 'standard'
            % 平面 2R：alpha = 0, d = 0
            param.dh_table = [
                0, param.l1, 0, 0;
                0, param.l2, 0, 0
            ];

            param.T_tool = eye(4);

        case 'craig'
            % 对于平面 2R，modified 与 standard 数值相同
            % 但语义不同（变换顺序不同）
            param.dh_table = [
                0, 0, 0, 0;
                0, param.l1, 0, 0
            ];

            param.T_tool = TransT('x', param.l2);

        otherwise
            error('Param2R:InvalidMethod', ...
                  'method 必须为 ''standard'' 或 ''modified''');
    end

    % ===== 记录 DH 类型 =====
    param.dh_method = method;

end