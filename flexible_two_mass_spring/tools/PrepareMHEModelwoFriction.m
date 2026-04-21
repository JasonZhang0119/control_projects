function model = PrepareMHEModelwoFriction(A, B)
    import casadi.*
    %% --- 1. 维度提取 ---
    nx = size(A, 1); % 5
    nw = 3; % 假设有3个独立的过程噪声 (i, v1, v2)

    %% --- 2. 定义符号变量 ---
    x = SX.sym('x', nx);
    w = SX.sym('w', nw);     % 过程噪声 (MHE的优化变量，类似于MPC的u)
    u_ext = SX.sym('u_ext'); % 实际控制电压 (作为参数p传入)
    x_dot = SX.sym('x_dot', nx);
    
    %% --- 3. 定义动力学方程 (含噪声项) ---
    % 噪声分配矩阵 G (将噪声注入电流和速度通道)
    G = zeros(nx, nw);
    G(1, 1) = 1; 
    G(3, 2) = 1; 
    G(5, 3) = 1; 
    
    f_expl = A*x + B*u_ext + G*w;
    
    %% --- 4. 封装成 acados_model ---
    model = AcadosModel();
    model.name = 'flexible_system_mhe';
    
    model.x = x;
    model.u = w;     % MHE 优化的是噪声序列
    model.p = u_ext; % 控制电压是已知外部参数
    model.xdot = x_dot;
    model.f_expl_expr = f_expl; 
    model.f_impl_expr = x_dot - f_expl; 
    
    %% --- 5. 定义测量输出表达式 ---
    % 假设物理测量 y 是负载端角度 theta_2
    % MHE 需要最小化: [测量残差; 噪声强度]
    model.cost_y_expr_0 = [
                x(4);
                w;
                x
            ];
    model.cost_y_expr = [
        x(4); % 状态中的 theta_2，将与实际测量的 yref 对比
        w     % 惩罚过程噪声，使其尽量接近 0
    ];
    
    model.cost_y_expr_e = [];
end