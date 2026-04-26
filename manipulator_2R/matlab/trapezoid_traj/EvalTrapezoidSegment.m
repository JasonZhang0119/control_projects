function [q, q_dot, q_ddot] = EvalTrapezoidSegment(seg, t)
%EVALTRAPEZOIDSEGMENT 单段梯形轨迹评估模板。
%
% 输入
% ----
% seg : struct
%     由 CreateMultiSegmentTrapezoidal 生成的段结构体。
%
% t : double
%     查询时刻。
%
% 输出
% ----
% q, q_dot, q_ddot : double
%     时刻 t 的位置、速度和加速度。

    if ~isfield(seg, 't0') || ~isfield(seg, 'tf')
        error('EvalTrapezoidSegment:InvalidSegment', ...
              'Segment must contain t0 and tf.');
    end

    if t <= seg.t0
        tau = 0;
    elseif t >= seg.tf
        tau = seg.tf - seg.t0;
    else
        tau = t - seg.t0;
    end

    % ============================================================
    % Step 1：读取段轮廓参数
    % ============================================================
    %
    % TODO:
    %   提取你存放在 seg.profile 中的轮廓参数。
    %   建议字段可能包括：
    %       t_acc, t_flat, t_dec, v_peak, a_max, direction, distance
    %
    %   示例：
    %       profile = seg.profile;
    %

    
    profile = seg.profile; 

    % ============================================================
    % Step 2：判断当前所处的运动阶段并计算轨迹
    % ============================================================
    %
    % TODO:
    %   判断 tau 当前处于：
    %       - 加速段
    %       - 匀速段
    %       - 减速段
    %
    %   然后计算：
    %       q(t), q_dot(t), q_ddot(t)
    %
    %   常见做法：
    %       1）先计算局部标量进度 s(t)
    %       2）再把 s(t) 映射到该段的关节向量方向上
    %
    q = [];
    q_dot = [];
    q_ddot = [];

    % TODO：把上面的占位符替换成真实公式。
end
