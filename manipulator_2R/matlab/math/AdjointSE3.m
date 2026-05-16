function AdT = AdjointSE3(T)
%ADJOINTSE3 计算 SE(3) 的 Adjoint 矩阵
%
% Parameters
% ----------
% T : double, size (4, 4)
%     齐次变换矩阵。
%
% Returns
% -------
% AdT : double, size (6, 6)
%     Adjoint 矩阵。假设 twist convention 为 V = [omega; v]。

    R = T(1:3, 1:3);
    p = T(1:3, 4);

    p_hat = VecToSo3(p);

    AdT = [
        R, zeros(3, 3);
        p_hat * R, R
    ];
end


function so3mat = VecToSo3(w)
%VECTOSO3 将三维向量转换为反对称矩阵

    so3mat = [
          0, -w(3),  w(2);
       w(3),     0, -w(1);
      -w(2),  w(1),     0
    ];
end