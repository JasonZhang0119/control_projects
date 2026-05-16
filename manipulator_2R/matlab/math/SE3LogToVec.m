function V = SE3LogToVec(T)
%SE3LOG 计算 SE(3) 的对数映射
%
% Parameters
% ----------
% T : double, size (4,4)
%     SE(3) 变换矩阵
%
% Returns
% -------
% V : double, size (6,1)
%     twist 向量 [omega; v]

    R = T(1:3,1:3);
    p = T(1:3,4);

    % ===== Step 1: SO(3) log =====
    theta = acos((trace(R)-1)/2);

    if abs(theta) < 1e-6
        % 纯平移
        omega = [0;0;0];
        v = p;
    else
        % 计算 skew(omega)
        omega_hat = (theta/(2*sin(theta))) * (R - R');

        % 提取 omega
        omega = [
            omega_hat(3,2);
            omega_hat(1,3);
            omega_hat(2,1)
        ];

        % ===== Step 2: 计算 G^{-1} =====
        omega_hat_sq = omega_hat * omega_hat;

        G_inv = (1/theta)*eye(3) ...
              - 0.5 * omega_hat ...
              + (1/theta - 0.5*cot(theta/2)) * omega_hat_sq;

        v = G_inv * p;
    end

    V = [omega; v];
end