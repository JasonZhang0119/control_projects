%% --- 1. 参数定义 ---
Ts = 0.001;     % 采样周期
N = 200;        % 预测步长 (Horizon)

%% --- 2. 调用函数生成模型 ---
model_wo_friction = PrepareModelwoFriction(A_num, B_num, B_num, C_num);
nx = size(A_num, 1);
nu = size(B_num, 2);

%% 2) 定义 OCP
ocp_wo_friction = AcadosOcp();
ocp_wo_friction.model = model_wo_friction;
Tf = N*Ts;
ocp_wo_friction.solver_options.N_horizon = N;
ocp_wo_friction.solver_options.tf = Tf;

% 离散化方式
ocp_wo_friction.solver_options.shooting_nodes = linspace(0, Tf, N+1);
ocp_wo_friction.solver_options.nlp_solver_type = 'SQP_RTI';
ocp_wo_friction.solver_options.qp_solver_warm_start = 1;
ocp_wo_friction.solver_options.nlp_solver_max_iter = 1;
ocp_wo_friction.solver_options.qp_solver_iter_max = 50;

simulink_opts = get_acados_simulink_opts;
simulink_opts.outputs.CPU_time_qp = 1;
ocp_wo_friction.simulink_opts = simulink_opts;

%% 3) 成本函数 - 修正维度 (y = [x; u])
ny = 2 + nu; 
ny_e = 2;

ocp_wo_friction.cost.cost_type   = 'NONLINEAR_LS';
ocp_wo_friction.cost.cost_type_e = 'NONLINEAR_LS';

% 权重矩阵 W
w_angle = 1; % 角度跟踪权重
w_u = 1;    % 控制量权重
ocp_wo_friction.cost.W = diag([w_angle, w_angle, w_u]); 
ocp_wo_friction.cost.W_e = 10 * diag([w_angle, w_angle]);

% 设置参考值 yref
theta_target = 1.5 * pi; % 假设目标是 270 度
ocp_wo_friction.cost.yref = [sin(theta_target); cos(theta_target); 0];
ocp_wo_friction.cost.yref_e = [sin(theta_target); cos(theta_target)];

%% 5) 约束 - 包含状态约束
% --- 输入约束 ---
umin = -48;
umax = 48;
ocp_wo_friction.constraints.lbu = umin;  
ocp_wo_friction.constraints.ubu = umax;  
ocp_wo_friction.constraints.idxbu = 0; % 假设控制输入索引为0

% --- 状态约束 (针对第一个状态 x1) ---
% 设定 x1 的范围，例如 [-0.5, 0.5]，请根据实际物理意义调整
x1_min = -20; 
x1_max =  20;

ocp_wo_friction.constraints.idxbx = 0; % 约束第一个状态变量
ocp_wo_friction.constraints.lbx = x1_min;
ocp_wo_friction.constraints.ubx = x1_max;

% 终端状态约束
ocp_wo_friction.constraints.idxbx_e = 0;
ocp_wo_friction.constraints.lbx_e = x1_min;
ocp_wo_friction.constraints.ubx_e = x1_max;

% --- 软约束 (Slacks) ---
% 强烈建议添加，否则当外部扰动过大导致 x1 稍微超出范围时，求解器会报 Status 4 (无解)
ocp_wo_friction.constraints.idxsbx = 0; % 对应 idxbx 的第一个元素进行软化
L_penalty = 1e4; % 二次惩罚权重
l_penalty = 1e3; % 线性惩罚权重
ocp_wo_friction.cost.Zl = L_penalty; 
ocp_wo_friction.cost.Zu = L_penalty;
ocp_wo_friction.cost.zl = l_penalty;
ocp_wo_friction.cost.zu = l_penalty;

% 终端步也同样软化
ocp_wo_friction.constraints.idxsbx_e = 0;
ocp_wo_friction.cost.Zl_e = L_penalty;
ocp_wo_friction.cost.Zu_e = L_penalty;
ocp_wo_friction.cost.zl_e = l_penalty;
ocp_wo_friction.cost.zu_e = l_penalty;

ocp_wo_friction.constraints.x0 = zeros(nx, 1);    % 初始状态

%% 3) 创建 solver_nonlinear
solver_nonlinear = AcadosOcpSolver(ocp_wo_friction);

%% Compile Sfunctions
cd c_generated_code
make_sfun; 
cd ..

%% ========= prepare observer ========
Bd = B_num;
Cd = zeros(1, 1);
Gp_aug_wo_friction = PrepareAugmentedModelDisturbanceObserver(sys_full_ss, Bd, Cd, 1, 2);

[eq_matrix, Nx, Nu] = DesignFeedforwardStateSpace(sys_full_ss, 2);

sys_full_ss_d = c2d(sys_full_ss, Ts);

% 针对电流追不准的情况，加大第一个状态的 Q 权重
Q_diag = [10, 1e-4, 1e-4, 1e-4, 1e-4]; % 给电流通道更大的权，迫使它通过角度残差来修正
R_val = 1e-5; % 假设角度传感器很准

% 计算离散 L
L = dlqe(sys_full_ss_d.A, eye(5), sys_full_ss_d.C, diag(Q_diag), R_val);


