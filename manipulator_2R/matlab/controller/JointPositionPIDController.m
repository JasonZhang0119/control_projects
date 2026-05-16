classdef JointPositionPIDController < handle
%JOINTPOSITIONPIDCONTROLLER 关节空间 PID 控制器（带内部状态）
%
% Notes
% -----
% - 内部维护积分状态 integral_
% - 支持前馈项（动力学补偿）
% - 支持限幅
%
% Methods
% -------
% - step(): 单步控制更新
% - reset(): 重置状态

    properties
        Kp
        Ki
        Kd
        Ts

        tau_min
        tau_max

        param   % robot param
    end

    properties (Access = private)
        integral_
    end

    methods
        function obj = JointPositionPIDController(pid_param, param, Ts, options)
        % 构造函数

            if nargin < 4
                options = struct();
            end

            % ==== 基本参数 ====
            obj.param = param;
            obj.Ts = Ts;

            n = numel(param.joint_types);

            % ==== 增益归一化 ====
            obj.Kp = obj.normalize_gain(pid_param.Kp, n, 'Kp');
            obj.Kd = obj.normalize_gain(pid_param.Kd, n, 'Kd');

            if isfield(pid_param, 'Ki')
                obj.Ki = obj.normalize_gain(pid_param.Ki, n, 'Ki');
            else
                obj.Ki = zeros(n, n);
            end

            % ==== 限幅 ====
            obj.tau_min = obj.get_option(options, 'tau_min', -inf(n,1));
            obj.tau_max = obj.get_option(options, 'tau_max',  inf(n,1));

            % ==== 初始化状态 ====
            obj.integral_ = zeros(n,1);
        end

        function tau = step(obj, q_ref, qd_ref, qdd_ref, q, qd)
        %STEP 单步控制
        %
        % Parameters
        % ----------
        % q_ref, qd_ref, qdd_ref : (n x 1)
        % q, qd : (n x 1)
        %
        % Returns
        % -------
        % tau : (n x 1)

            % 向量化
            q_ref  = q_ref(:);
            qd_ref = qd_ref(:);
            qdd_ref = qdd_ref(:);
            q      = q(:);
            qd     = qd(:);

            % ==== 误差 ====
            e_q  = q_ref - q;
            e_qd = qd_ref - qd;

            % ==== 积分更新 ====
            obj.integral_ = obj.integral_ + e_q * obj.Ts;

            % ==== PID ====
            tau_fb = obj.Kp * e_q ...
                   + obj.Kd * e_qd ...
                   + obj.Ki * obj.integral_;

            % ==== 前馈 ====
            [M, h, G] = Robot2R_Dynamics(q, qd, obj.param);
            tau_ff_1 = h + G;
            tau_ff_2 = M * qdd_ref;

            % ==== 合成 ====
            tau_raw = M * tau_fb + tau_ff_1 + tau_ff_2; 

            % ==== 限幅 ====
            tau = min(max(tau_raw, obj.tau_min), obj.tau_max);
        end

        function reset(obj)
        %RESET 重置积分器
            obj.integral_(:) = 0;
        end
    end

    methods (Access = private)
        function K_mat = normalize_gain(~, K, n, name)
            if isscalar(K)
                K_mat = K * eye(n);
            elseif isvector(K) && numel(K) == n
                K_mat = diag(K(:));
            elseif isequal(size(K), [n, n])
                K_mat = K;
            else
                error('%s 尺寸错误', name);
            end
        end

        function value = get_option(~, options, field_name, default_value)
            if isfield(options, field_name)
                value = options.(field_name);
            else
                value = default_value;
            end
        end
    end
end