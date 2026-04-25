function q = ik_2R_continuous(p, l1, l2, q_prev)
%IK_2R_CONTINUOUS 平面 2R 机械臂解析逆运动学，并优先选择与上一时刻更接近的解
%
% Parameters
% ----------
% p : (1,2) double
%     目标笛卡尔位置 [x, y]
% l1 : double
%     第一连杆长度
% l2 : double
%     第二连杆长度
% q_prev : (1,2) double or []
%     上一个时刻的关节解；若为空，则默认返回第一组可行解
%
% Returns
% -------
% q : (1,2) double
%     选中的关节解 [q1, q2]

    x = p(1);
    y = p(2);

    c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
    c2 = min(max(c2, -1), 1);

    s2_pos = sqrt(1 - c2^2);
    s2_neg = -sqrt(1 - c2^2);

    q2_a = atan2(s2_pos, c2);
    q2_b = atan2(s2_neg, c2);

    q1_a = atan2(y, x) - atan2(l2*sin(q2_a), l1 + l2*cos(q2_a));
    q1_b = atan2(y, x) - atan2(l2*sin(q2_b), l1 + l2*cos(q2_b));

    qa = [wrapToPiLocal(q1_a), wrapToPiLocal(q2_a)];
    qb = [wrapToPiLocal(q1_b), wrapToPiLocal(q2_b)];

    if isempty(q_prev)
        q = qa;
    else
        da = norm(angleDiffVec(qa, q_prev));
        db = norm(angleDiffVec(qb, q_prev));

        if da <= db
            q = qa;
        else
            q = qb;
        end
    end
end

function d = angleDiffVec(a, b)
%ANGLEDIFFVEC 计算两个关节角向量的最小角差
    d = [wrapToPiLocal(a(1)-b(1)), wrapToPiLocal(a(2)-b(2))];
end

function a = wrapToPiLocal(a)
%WRAPTOPILOCAL 将角度包裹到 [-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end