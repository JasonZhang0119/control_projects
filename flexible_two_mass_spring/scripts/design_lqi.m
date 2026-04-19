%% --- 1. 准备增广模型 ---
% 假设 A, B, C 是你的五阶离散/连续模型 (这里推荐先在连续域设计再离散化)
nu = size(B_num, 2);
nx = size(A_num, 1); 
C_y = [0, 0, 0, 1, 0]; % 测量 theta_2

A_lqi = [A_num, zeros(nx, 1); 
         C_y,   0];
B_lqi = [B_num; 0];

%% --- 2. 权重矩阵设计 (复用 MPC 的成功经验) ---
% 状态顺序: [i, th1, v1, th2, v2, integral_error]
w_i      = 100;
w_theta1 = 10;
w_v1     = 20;     % 电机端阻尼
w_theta2 = 100;    % 输出端位置
w_v2     = 100;    % 柔性端阻尼 (压制 200 rad/s)
w_int    = 1;    % 积分项权重 (LQI 消除静差的核心)

Q_lqi = diag([w_i, w_theta1, w_v1, w_theta2, w_v2, w_int]);
R_lqi = 100;      % 电压惩罚

%% --- 3. 计算反馈增益 K ---
% K 是一个 1x6 的向量: [K_state, K_integral]
K_lqi = lqi(ss(A_num, B_num, C_y, 0), Q_lqi, R_lqi); 
% 或者使用增广矩阵计算: K_lqi = lqr(A_lqi, B_lqi, Q_lqi, R_lqi);

Kx = K_lqi(1:5);  % 状态反馈增益
Ki = K_lqi(6);    % 积分增益

%% ========= 6. 观测器设计 (Observer) ========
Bd = B_num;
Cd = zeros(1, 1);
% 修正：测量矩阵 C 不能为 0。假设测量的是 theta_2 (状态向量第 4 个)
Cd_actual = [0, 0, 0, 1, 0]; 
sys_full_ss.C = Cd_actual; % 确保原始系统的 C 正确

% 离散化系统
sys_full_ss_d = c2d(sys_full_ss, Ts);

% 观测器权重：加大第一个状态（电流）的权重 Q，增强对其动态的追随
Q_diag = [100, 1e-6, 1e-4, 1e-6, 1e-4]; 
R_val = 1e-5; 

% 计算离散观测器增益 L (离散卡尔曼滤波设计)
L = dlqe(sys_full_ss_d.A, eye(5), sys_full_ss_d.C, diag(Q_diag), R_val);

Bd = B_num;
Cd = zeros(1, 1);

[eq_matrix, Nx, Nu] = DesignFeedforwardStateSpace(sys_full_ss, 2);