function q = IK2R_Continuous(p, param, q_prev)
%IK2R_CONTINUOUS 平面 2R 机械臂连续逆运动学模板。
%
% 这个模板保留了分支选择逻辑，并将核心解析逆解部分留作 TODO，
% 方便你后续自行补全。
%
% 输入
% ----
% p : double, size (2,1) or (1,2)
%     末端位置 [x; y]
%
% param : struct
%     机器人参数结构体，期望包含：
%         l1, l2
%
% q_prev : double, size (2,1) or (1,2), optional
%     上一时刻的关节解；如果为空，则返回默认分支。
%
% 输出
% ----
% q : double, size (2,1)
%     选中的连续逆解。

    if nargin < 3
        q_prev = [];
    end

    p = p(:);

    if numel(p) ~= 2
        error('IK2R_Continuous:DimensionMismatch', ...
              'p must be a 2-element position vector.');
    end

    % ============================================================
    % Step 1：计算两组解析逆解分支
    % ============================================================
    %
    q_candidate = IK2R_Analytic(p, param);
    q_a = q_candidate(:, 1);
    q_b = q_candidate(:, 2);

    % ============================================================
    % Step 2：选择与上一时刻最接近的分支
    % ============================================================

    if isempty(q_prev)
        q = q_a;  % 默认分支，可按需要改成 q_b
    else
        q_prev = q_prev(:);

        if numel(q_prev) ~= 2
            error('IK2R_Continuous:PrevDimensionMismatch', ...
                  'q_prev must be a 2-element joint vector.');
        end

        dist_a = angleDiffVec(q_a, q_prev);
        dist_b = angleDiffVec(q_b, q_prev);

        if dist_a' * dist_a <= dist_b' * dist_b
            q = q_a; 
        else
            q = q_b;
        end
    end
end

% -------------------------------------------------------------------------
% 局部函数：将角度包裹到 [-pi, pi]
% -------------------------------------------------------------------------
function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

% -------------------------------------------------------------------------
% 局部函数：计算包裹后的角度差
% -------------------------------------------------------------------------
function d = angleDiffVec(a, b)
    d = [wrapToPiLocal(a(1) - b(1)); wrapToPiLocal(a(2) - b(2))];
end
