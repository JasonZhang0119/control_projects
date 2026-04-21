
%% --- 1. 参数定义 ---
Ts_mhe = Ts;     % 采样周期需与控制器一致
N_mhe = 50;          % MHE 窗口长度 (通常比 MPC 短，以保证 1kHz 实时性)

%% --- 2. 调用函数生成模型 ---
% 注意：此处调用专门为 MHE 编写的模型函数，其中 u 为过程噪声 w，p 为实际控制电压
model_mhe = PrepareMHEModelwoFriction(A_num, B_num); 
nx = size(A_num, 1); % 5
nw = 3;              % 过程噪声维度 (针对 i, v1, v2)
ny = 1 + nw;
ny_0 = 1 + nw + nx;

%% --- 3. 定义 MHE OCP ---
ocp_mhe = AcadosOcp();
ocp_mhe.model = model_mhe;
Tf_mhe = N_mhe * Ts_mhe;
ocp_mhe.solver_options.N_horizon = N_mhe;
ocp_mhe.solver_options.tf = Tf_mhe;
% 求解器设置：SQP_RTI 是 1kHz 运行的关键
ocp_mhe.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp_mhe.solver_options.nlp_solver_type = 'SQP_RTI'; 
ocp_mhe.solver_options.hessian_approx = 'GAUSS_NEWTON'; 
ocp_mhe.solver_options.qp_solver_warm_start = 1;
ocp_mhe.solver_options.nlp_solver_max_iter = 1; 

%% --- 4. 成本函数设计 (最小化残差与噪声) ---
% y_mhe = [theta2_model; w_i; w_v1; w_v2] (维度: 1 + 3 = 4)
ocp_mhe.cost.cost_type   = 'NONLINEAR_LS';
ocp_mhe.cost.cost_type_0 = 'NONLINEAR_LS'; 
% --- 权重矩阵 W 设计 ---
r_meas = 1e5;    % 测量值 theta2 的权重 (值越大越相信传感器数据)
q_wi   = 0.1;    % 电流过程噪声权重
q_wv1  = 0.1;    % 电机端速度噪声权重
q_wv2  = 0.1;    % 负载端速度噪声权重
% W 对应窗口内的阶段代价 l(y, u) [cite: 25, 45]
ocp_mhe.cost.W = diag([r_meas, q_wi, q_wv1, q_wv2]);
% --- 到达代价 (Arrival Cost) ---
% W_e 对应文档中的 V_f，代表对窗口最老状态估计的信心 [cite: 77, 131, 145]
% 通常通过 EKF 的协方差矩阵 P 的逆来动态更新
ocp_mhe.cost.W_0 = eye(ny_0); 
% 参考值 (yref 在运行期间通过参数实时更新，此处初始化)
ocp_mhe.cost.yref = zeros(4, 1); 

%% --- 5. 状态硬约束 (解决饱和的关键) ---
% 显式设定状态约束集合 X [cite: 80, 96]
i_max_limit = 10; % 物理电流限幅 10A
Jbx = zeros(1, nx);
Jbx(1) = 1;
ocp_mhe.constraints.idxbx = 0; % 约束 x(1)
ocp_mhe.constraints.lbx = -i_max_limit;
ocp_mhe.constraints.ubx = i_max_limit;
Jbx_e = zeros(1, nx);
Jbx_e(1) = 1;
ocp_mhe.constraints.idxbx_e = 0;
ocp_mhe.constraints.lbx_e = -i_max_limit;
ocp_mhe.constraints.ubx_e =  i_max_limit;

%% --- Simulink 仿真配置 ----
simulink_opts_mhe = get_acados_simulink_opts;
simulink_opts_mhe.outputs.CPU_time_qp = 1;
simulink_opts_mhe.outputs.xtraj = 1;
simulink_opts_mhe.inputs.y_ref_0 = 1;
simulink_opts_mhe.inputs.y_ref = 1; % 允许在 Simulink 中动态输入 Reference
simulink_opts_mhe.inputs.cost_W = 1;
simulink_opts_mhe.inputs.cost_W_0 = 1;
simulink_opts_mhe.inputs.parameter_traj = 1;
ocp_mhe.simulink_opts = simulink_opts_mhe;

%% --- 6. 创建 Solver 并编译 ---
solver_mhe = AcadosOcpSolver(ocp_mhe);
cd c_generated_code
make_sfun; % 生成 Simulink 调用的 S-Function
cd ..
%% 动态参数设置
r_meas = 1e4;    % 测量值 theta2 的权重 (值越大越相信传感器数据)
q_wi   = 0.01;   % 电流过程噪声权重
q_wv1  = 1;    % 电机端速度噪声权重
q_wv2  = 1;    % 负载端速度噪声权重
% 
Q_diag_val = diag([r_meas, q_wi, q_wv1, q_wv2]); % 温度权重
p_i   = 0.01;
p_th1 = 0.1;
p_v1  = 0.1;
p_th2 = 1e4;
p_v2  = 0.1;
P0 = diag([p_i, p_th1, p_v1, p_th2, p_v2]);
MHE.W = Q_diag_val;
MHE.W0 = blkdiag(Q_diag_val, P0);