classdef EndImpedanceControl < handle
%ENDIMPEDANCECONTROL 任务空间力到位置生成器骨架
%
% 本类目前只保留最小功能：
% - 输入：任务空间力指令 Fcmd
% - 输入：当前实际状态 x, x_dot, x_ddot
% - 输入：当前期望状态 xd, xd_dot, xd_ddot
% - 输出：更新后的期望位置 xd
%
% 说明
% ----
% - 从接口角度看，这个类更像“力到位置”的生成器，而不是经典的
%   力矩级阻抗控制器。
%   阻尼模型。

    properties
        Md
        Dd
        Kd
        Ts

        param
        options
    end

    properties (Access = private)
        xd_
        xd_dot_
        xd_ddot_

        xd_init_
        xd_dot_init_
        xd_ddot_init_
    end

    methods
        function obj = EndImpedanceControl(impedance_param, param, Ts, options)
        %ENDIMPEDANCECONTROL 构造函数
        %
        % 参数
        % ----
        % impedance_param : struct
        %     必需字段：
        %         Md : scalar / m x 1 / m x m
        %         Dd : scalar / m x 1 / m x m
        %         Kd : scalar / m x 1 / m x m
        %
        % param : struct
        %     机器人参数结构体。
        %
        % Ts : double
        %     采样时间。
        %
        % options : struct, 可选
        %     控制器选项：
        %         task_dim : 任务空间维度，默认 2
        %         x0       : 初始期望位置，默认 zeros(task_dim,1)
        %         x_dot0    : 初始期望速度，默认 zeros(task_dim,1)
        %         x_ddot0   : 初始期望加速度，默认 zeros(task_dim,1)

            if nargin < 4
                options = struct();
            end

            if Ts <= 0
                error('EndImpedanceControl:InvalidSampleTime', ...
                      'Ts 必须为正数。');
            end

            if ~isfield(param, 'joint_types')
                error('EndImpedanceControl:MissingField', ...
                      'param.joint_types 缺失。');
            end

            if ~isfield(impedance_param, 'Md')
                error('EndImpedanceControl:MissingField', ...
                      'impedance_param.Md 缺失。');
            end

            if ~isfield(impedance_param, 'Dd')
                error('EndImpedanceControl:MissingField', ...
                      'impedance_param.Dd 缺失。');
            end

            if ~isfield(impedance_param, 'Kd')
                error('EndImpedanceControl:MissingField', ...
                      'impedance_param.Kd 缺失。');
            end

            obj.param = param;
            obj.Ts = Ts;
            obj.options = options;

            m = obj.get_option(options, 'task_dim', 2);

            obj.Md = obj.normalize_gain(impedance_param.Md, m, 'Md');
            obj.Dd = obj.normalize_gain(impedance_param.Dd, m, 'Dd');
            obj.Kd = obj.normalize_gain(impedance_param.Kd, m, 'Kd');

            x0 = obj.get_option(options, 'x0', zeros(m, 1));
            x_dot0 = obj.get_option(options, 'x_dot0', zeros(m, 1));
            x_ddot0 = obj.get_option(options, 'x_ddot0', zeros(m, 1));

            obj.xd_ = x0(:);
            obj.xd_dot_ = x_dot0(:);
            obj.xd_ddot_ = x_ddot0(:);

            if numel(obj.xd_) ~= m
                error('EndImpedanceControl:InvalidInitSize', ...
                      'options.x0 必须是 %d x 1 向量。', m);
            end

            if numel(obj.xd_dot_) ~= m
                error('EndImpedanceControl:InvalidInitSize', ...
                      'options.x_dot0 必须是 %d x 1 向量。', m);
            end

            if numel(obj.xd_ddot_) ~= m
                error('EndImpedanceControl:InvalidInitSize', ...
                      'options.x_ddot0 必须是 %d x 1 向量。', m);
            end

            obj.xd_init_ = obj.xd_;
            obj.xd_dot_init_ = obj.xd_dot_;
            obj.xd_ddot_init_ = obj.xd_ddot_;
        end

        function [Fcmd, info] = step(obj, x, x_dot, x_ddot, xd, xd_dot, xd_ddot)
        %STEP 更新期望任务空间位置
        %
        % 参数
        % ----
        % Fcmd : double, size (m, 1)
        %     任务空间力指令。
        %
        % x : double, size (m, 1)
        %     当前实际位置。
        %
        % x_dot : double, size (m, 1)
        %     当前实际速度。
        %
        % x_ddot : double, size (m, 1)
        %     当前实际加速度。
        %
        % xd : double, size (m, 1)
        %     当前期望位置。
        %
        % xd_dot : double, size (m, 1)
        %     当前期望速度。
        %
        % xd_ddot : double, size (m, 1)
        %     当前期望加速度。
        %
        % 返回
        % ----
        % xd_out : double, size (m, 1)
        %     发给下层位置控制器的期望位置。
        %
        % info : struct
        %     调试信息。

            x = x(:);
            x_dot = x_dot(:);
            x_ddot = x_ddot(:);
            xd = xd(:);
            xd_dot = xd_dot(:);
            xd_ddot = xd_ddot(:);

            m = numel(obj.xd_);

            obj.check_dimension(x, m, 'x');
            obj.check_dimension(x_dot, m, 'x_dot');
            obj.check_dimension(x_ddot, m, 'x_ddot');
            obj.check_dimension(xd, m, 'xd');
            obj.check_dimension(xd_dot, m, 'xd_dot');
            obj.check_dimension(xd_ddot, m, 'xd_ddot');

            % ============================================================
            % 离散时间阻抗 / 导纳更新
            % ============================================================
            % 你可以在这里实现类似如下的逻辑：
            %
            %
            % 也可以加入：
            % - 低通滤波
            % - 位置/速度限幅
            % - 速度/加速度限幅
            % - 状态切换或复位逻辑

            xd_out = xd;

            obj.xd_ = xd_out;
            obj.xd_dot_ = xd_dot;
            obj.xd_ddot_ = xd_ddot;

            Fcmd = obj.Md * (x_ddot - obj.xd_ddot_) + obj.Dd * (x_dot - obj.xd_dot_) + obj.Kd * (x - obj.xd_);

            info = struct();
            info.Fcmd = Fcmd;
            info.x = x;
            info.x_dot = x_dot;
            info.x_ddot = x_ddot;
            info.xd_input = xd;
            info.xd_dot_input = xd_dot;
            info.xd_ddot_input = xd_ddot;
            info.xd = xd_out;
            info.xd_dot = obj.xd_dot_;
            info.xd_ddot = obj.xd_ddot_;
            info.todo = true;
        end

        function set_state(obj, xd, xd_dot, xd_ddot)
        %SET_STATE 手动设置内部期望状态

            xd = xd(:);
            xd_dot = xd_dot(:);
            xd_ddot = xd_ddot(:);

            obj.check_dimension(xd, numel(obj.xd_), 'xd');
            obj.check_dimension(xd_dot, numel(obj.xd_dot_), 'xd_dot');
            obj.check_dimension(xd_ddot, numel(obj.xd_ddot_), 'xd_ddot');

            obj.xd_ = xd;
            obj.xd_dot_ = xd_dot;
            obj.xd_ddot_ = xd_ddot;
        end

        function reset(obj)
        %RESET 重置内部期望状态
            obj.xd_ = obj.xd_init_;
            obj.xd_dot_ = obj.xd_dot_init_;
            obj.xd_ddot_ = obj.xd_ddot_init_;
        end
    end

    methods (Access = private)
        function check_dimension(~, value, expected_n, name)
        %CHECK_DIMENSION 检查向量维度是否匹配
            if numel(value) ~= expected_n
                error('EndImpedanceControl:DimensionMismatch', ...
                      '%s 维度必须为 %d x 1。', name, expected_n);
            end
        end

        function K_mat = normalize_gain(~, K, n, name)
        %NORMALIZE_GAIN 将增益归一化为 n x n 矩阵
            if isscalar(K)
                K_mat = K * eye(n);
            elseif isvector(K) && numel(K) == n
                K_mat = diag(K(:));
            elseif isequal(size(K), [n, n])
                K_mat = K;
            else
                error('EndImpedanceControl:InvalidGainSize', ...
                      '%s 必须是标量、n x 1 向量或 n x n 矩阵。', name);
            end
        end

        function value = get_option(~, options, field_name, default_value)
        %GET_OPTION 读取可选字段，若不存在则返回默认值
            if isfield(options, field_name)
                value = options.(field_name);
            else
                value = default_value;
            end
        end
    end
end
