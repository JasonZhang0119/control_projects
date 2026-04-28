classdef JointForcePIDController < handle
%JOINTFORCEPIDCONTROLLER 关节空间/任务空间力控 PID 控制器模板
%
% Notes
% -----
% - 用于基于末端力误差的 force control。
% - 内部维护 force error 的积分状态。
% - 核心逻辑留在 TODO 中自行填写。
%
% Typical Form
% ------------
%   F_cmd = Kp * e_F + Ki * int(e_F) + Kd * e_F_dot
%   tau   = J(q)' * F_cmd + optional_compensation
%
% Methods
% -------
% - step()
% - reset()

    properties
        Kp
        Ki
        Kd
        Ts

        tau_min
        tau_max

        param
        options
    end

    properties (Access = private)
        integral_
        prev_error_
    end

    methods
        function obj = JointForcePIDController(force_pid_param, param, Ts, options)
        %JOINTFORCEPIDCONTROLLER 构造函数
        %
        % Parameters
        % ----------
        % force_pid_param : struct
        %     力控 PID 参数：
        %         Kp : scalar / m x 1 / m x m
        %         Ki : scalar / m x 1 / m x m, optional
        %         Kd : scalar / m x 1 / m x m, optional
        %
        % param : struct
        %     机器人参数结构体。
        %
        % Ts : double
        %     采样时间。
        %
        % options : struct, optional
        %     控制器配置：
        %         task_dim : 控制的任务空间维度，默认 2
        %         tau_min  : 关节力矩下限
        %         tau_max  : 关节力矩上限
        %         use_gravity_compensation : 是否使用重力补偿
        %         use_coriolis_compensation : 是否使用科氏/离心补偿

            if nargin < 4
                options = struct();
            end

            if Ts <= 0
                error('JointForcePIDController:InvalidSampleTime', ...
                      'Ts 必须为正数。');
            end

            if ~isfield(param, 'joint_types')
                error('JointForcePIDController:MissingField', ...
                      'param.joint_types 缺失。');
            end

            obj.param = param;
            obj.Ts = Ts;
            obj.options = options;

            n = numel(param.joint_types);
            m = obj.get_option(options, 'task_dim', 2);

            if ~isfield(force_pid_param, 'Kp')
                error('JointForcePIDController:MissingPIDField', ...
                      'force_pid_param.Kp 缺失。');
            end

            obj.Kp = obj.normalize_gain(force_pid_param.Kp, m, 'Kp');

            if isfield(force_pid_param, 'Ki')
                obj.Ki = obj.normalize_gain(force_pid_param.Ki, m, 'Ki');
            else
                obj.Ki = zeros(m, m);
            end

            if isfield(force_pid_param, 'Kd')
                obj.Kd = obj.normalize_gain(force_pid_param.Kd, m, 'Kd');
            else
                obj.Kd = zeros(m, m);
            end

            obj.tau_min = obj.get_option(options, 'tau_min', -inf(n,1));
            obj.tau_max = obj.get_option(options, 'tau_max',  inf(n,1));

            obj.tau_min = obj.tau_min(:);
            obj.tau_max = obj.tau_max(:);

            if numel(obj.tau_min) ~= n || numel(obj.tau_max) ~= n
                error('JointForcePIDController:InvalidLimitSize', ...
                      'tau_min / tau_max 必须为 n x 1。');
            end

            obj.integral_ = zeros(m, 1);
            obj.prev_error_ = zeros(m, 1);
        end

        function [tau, info] = step(obj, F_ref, F_meas, q, qd)
        %STEP 单步力控更新
        %
        % Parameters
        % ----------
        % F_ref : double, size (m, 1)
        %     期望任务空间力。
        %
        % F_meas : double, size (m, 1)
        %     测量任务空间力。
        %
        % q : double, size (n, 1)
        %     当前关节位置。
        %
        % qd : double, size (n, 1)
        %     当前关节速度。
        %
        % Returns
        % -------
        % tau : double, size (n, 1)
        %     关节力矩输出。
        %
        % info : struct
        %     调试信息。

            F_ref = F_ref(:);
            F_meas = F_meas(:);
            q = q(:);
            qd = qd(:);

            n = numel(q);
            m = numel(F_ref);

            if numel(F_meas) ~= m
                error('JointForcePIDController:DimensionMismatch', ...
                      'F_meas 维度必须与 F_ref 一致。');
            end

            if numel(qd) ~= n
                error('JointForcePIDController:DimensionMismatch', ...
                      'qd 维度必须与 q 一致。');
            end

            if size(obj.Kp, 1) ~= m
                error('JointForcePIDController:TaskDimMismatch', ...
                      'F_ref 维度与控制器 task_dim 不一致。');
            end

            % ============================================================
            % Step 1: 力误差
            % ============================================================
            e_F = F_ref - F_meas;

            % ============================================================
            % Step 2: 积分与微分误差
            % ============================================================
            obj.integral_ = obj.integral_ + e_F * obj.Ts;
            e_F_dot = (e_F - obj.prev_error_) / obj.Ts;

            % ============================================================
            % Step 3: TODO 任务空间力控制律
            % ============================================================
            % TODO:
            % - 根据 e_F, integral_, e_F_dot 计算 F_cmd。
            % - 典型形式：
            %       F_cmd = Kp*e_F + Ki*integral_ + Kd*e_F_dot
            %
            F_cmd = zeros(m, 1);

            % ============================================================
            % Step 4: TODO Jacobian
            % ============================================================
            % TODO:
            % - 计算任务空间 Jacobian。
            % - 对 2R 平面位置力控，J 应为 2 x n。
            %
            J = zeros(m, n);

            % ============================================================
            % Step 5: TODO 力到关节力矩映射
            % ============================================================
            % TODO:
            % - 使用 tau_force = J' * F_cmd。
            %
            tau_force = zeros(n, 1);

            % ============================================================
            % Step 6: TODO 动力学补偿项
            % ============================================================
            % TODO:
            % - 可选加入重力补偿 G。
            % - 可选加入科氏/离心项 h。
            %
            tau_comp = zeros(n, 1);

            % ============================================================
            % Step 7: 合成与限幅
            % ============================================================
            tau_raw = tau_force + tau_comp;
            tau = min(max(tau_raw, obj.tau_min), obj.tau_max);

            obj.prev_error_ = e_F;

            % ============================================================
            % Step 8: 调试信息
            % ============================================================
            info = struct();
            info.e_F = e_F;
            info.e_F_dot = e_F_dot;
            info.integral = obj.integral_;
            info.F_cmd = F_cmd;
            info.tau_force = tau_force;
            info.tau_comp = tau_comp;
            info.tau_raw = tau_raw;
            info.tau = tau;
        end

        function reset(obj)
        %RESET 重置控制器内部状态
            obj.integral_(:) = 0;
            obj.prev_error_(:) = 0;
        end
    end

    methods (Access = private)
        function K_mat = normalize_gain(~, K, n, name)
        %NORMALIZE_GAIN 统一增益为 n x n 矩阵
            if isscalar(K)
                K_mat = K * eye(n);
            elseif isvector(K) && numel(K) == n
                K_mat = diag(K(:));
            elseif isequal(size(K), [n, n])
                K_mat = K;
            else
                error('JointForcePIDController:InvalidGainSize', ...
                      '%s 尺寸必须为 scalar / n x 1 / n x n。', name);
            end
        end

        function value = get_option(~, options, field_name, default_value)
        %GET_OPTION 读取配置项
            if isfield(options, field_name)
                value = options.(field_name);
            else
                value = default_value;
            end
        end
    end
end