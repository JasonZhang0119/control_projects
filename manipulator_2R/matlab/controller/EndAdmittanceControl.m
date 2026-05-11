classdef EndAdmittanceControl < handle
%ENDADMITTANCECONTROL 任务空间导纳控制器骨架
%
% 本类目前只保留最小功能：
% - 输入：外界力 Fext
% - 输入：当前实际状态 x, xdot, xddot
% - 输出：更新后的期望位置 xd
%
% 说明
% ----
% - 从控制结构上看，导纳控制更适合“力输入 -> 位置输出”。

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
        xdotd_
        xddotd_

        xd_init_
        xdotd_init_
        xddotd_init_
    end

    methods
        function obj = EndAdmittanceControl(admittance_param, param, Ts, options)
        %ENDADMITTANCECONTROL 构造函数
        %
        % 参数
        % ----
        % admittance_param : struct
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
        %         xdot0    : 初始期望速度，默认 zeros(task_dim,1)
        %         xddot0   : 初始期望加速度，默认 zeros(task_dim,1)

            if nargin < 4
                options = struct();
            end

            if Ts <= 0
                error('EndAdmittanceControl:InvalidSampleTime', ...
                      'Ts 必须为正数。');
            end

            if ~isfield(param, 'joint_types')
                error('EndAdmittanceControl:MissingField', ...
                      'param.joint_types 缺失。');
            end

            if ~isfield(admittance_param, 'Md')
                error('EndAdmittanceControl:MissingField', ...
                      'admittance_param.Md 缺失。');
            end

            if ~isfield(admittance_param, 'Dd')
                error('EndAdmittanceControl:MissingField', ...
                      'admittance_param.Dd 缺失。');
            end

            if ~isfield(admittance_param, 'Kd')
                error('EndAdmittanceControl:MissingField', ...
                      'admittance_param.Kd 缺失。');
            end

            obj.param = param;
            obj.Ts = Ts;
            obj.options = options;

            m = obj.get_option(options, 'task_dim', 2);

            obj.Md = obj.normalize_gain(admittance_param.Md, m, 'Md');
            obj.Dd = obj.normalize_gain(admittance_param.Dd, m, 'Dd');
            obj.Kd = obj.normalize_gain(admittance_param.Kd, m, 'Kd');

            x0 = obj.get_option(options, 'x0', zeros(m, 1));
            xdot0 = obj.get_option(options, 'xdot0', zeros(m, 1));
            xddot0 = obj.get_option(options, 'xddot0', zeros(m, 1));

            obj.xd_ = x0(:);
            obj.xdotd_ = xdot0(:);
            obj.xddotd_ = xddot0(:);

            if numel(obj.xd_) ~= m
                error('EndAdmittanceControl:InvalidInitSize', ...
                      'options.x0 必须是 %d x 1 向量。', m);
            end

            if numel(obj.xdotd_) ~= m
                error('EndAdmittanceControl:InvalidInitSize', ...
                      'options.xdot0 必须是 %d x 1 向量。', m);
            end

            if numel(obj.xddotd_) ~= m
                error('EndAdmittanceControl:InvalidInitSize', ...
                      'options.xddot0 必须是 %d x 1 向量。', m);
            end

            obj.xd_init_ = obj.xd_;
            obj.xdotd_init_ = obj.xdotd_;
            obj.xddotd_init_ = obj.xddotd_;
        end

        function [xd_out, info] = step(obj, Fext, x, xdot, xddot)
        %STEP 更新期望任务空间位置
        %
        % 参数
        % ----
        % Fext : double, size (m, 1)
        %     外界作用在机器人上的任务空间力。
        %
        % x : double, size (m, 1)
        %     当前实际位置。
        %
        % xdot : double, size (m, 1)
        %     当前实际速度。
        %
        % xddot : double, size (m, 1)
        %     当前实际加速度。
        %
        % 返回
        % ----
        % xd_out : double, size (m, 1)
        %     发给下层位置控制器的期望位置。
        %
        % info : struct
        %     调试信息。

            Fext = Fext(:);
            x = x(:);
            xdot = xdot(:);
            xddot = xddot(:);

            m = numel(obj.xd_);

            obj.check_dimension(Fext, m, 'Fext');
            obj.check_dimension(x, m, 'x');
            obj.check_dimension(xdot, m, 'xdot');
            obj.check_dimension(xddot, m, 'xddot');

            % ============================================================
            % TODO: 离散时间导纳更新
            % ============================================================
            % 你可以在这里实现类似如下的逻辑：
            %
            %   e_x = obj.xd_ - x;
            %   xddot_d = xddot + obj.Md \ (Fext ...
            %              - obj.Dd * (obj.xdotd_ - xdot) ...
            %              - obj.Kd * e_x);
            %   obj.xdotd_ = obj.xdotd_ + obj.Ts * xddot_d;
            %   obj.xd_    = obj.xd_    + obj.Ts * obj.xdotd_;
            %
            % 也可以加入：
            % - 低通滤波
            % - 位置/速度/加速度限幅
            % - 状态切换或复位逻辑

            xd_out = obj.xd_;
            

            info = struct();
            info.Fext = Fext;
            info.x = x;
            info.xdot = xdot;
            info.xddot = xddot;
            info.xd = obj.xd_;
            info.xdotd = obj.xdotd_;
            info.xddotd = obj.xddotd_;
            info.todo = true;
        end

        function set_state(obj, xd, xdotd, xddotd)
        %SET_STATE 手动设置内部期望状态

            xd = xd(:);
            xdotd = xdotd(:);
            xddotd = xddotd(:);

            obj.check_dimension(xd, numel(obj.xd_), 'xd');
            obj.check_dimension(xdotd, numel(obj.xdotd_), 'xdotd');
            obj.check_dimension(xddotd, numel(obj.xddotd_), 'xddotd');

            obj.xd_ = xd;
            obj.xdotd_ = xdotd;
            obj.xddotd_ = xddotd;
        end

        function reset(obj)
        %RESET 重置内部期望状态
            obj.xd_ = obj.xd_init_;
            obj.xdotd_ = obj.xdotd_init_;
            obj.xddotd_ = obj.xddotd_init_;
        end
    end

    methods (Access = private)
        function check_dimension(~, value, expected_n, name)
        %CHECK_DIMENSION 检查向量维度是否匹配
            if numel(value) ~= expected_n
                error('EndAdmittanceControl:DimensionMismatch', ...
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
                error('EndAdmittanceControl:InvalidGainSize', ...
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
