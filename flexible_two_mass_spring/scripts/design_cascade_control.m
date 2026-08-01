%% 串级控制器设计脚本 (基于分离模型定义)
% ========================================================
% 依赖：需先运行建模脚本以生成 sys_elec_tf, sys_mech_ss 等
% ========================================================

s = tf('s');
CTRL = struct();

%% 1. 电流环设计 (Inner Loop: U -> i)
% ========================================================
target_bw_cur = 10000; % 设定电流环带宽 (rad/s)

% 使用SIMC方法进行调参
[Kp, Ki] = DesignPID_0_0_2(961.5/307.7, 0, 1/307.7, 0, 1e-4);

% CTRL.Cur.Kp = Kp; 
% CTRL.Cur.Ki = Ki;

% 使用零极点对消方法
CTRL.Cur.Kp = L_val * target_bw_cur;
CTRL.Cur.Ki = R_val / L_val * CTRL.Cur.Kp;

CTRL.Cur.Ke = k_E_val; %通过前馈消除反电动势

C_cur = CTRL.Cur.Kp + CTRL.Cur.Ki/s;

% 电流环闭环 (U_ref -> i)
L_cur = C_cur * sys_elec_tf;
sys_cur_closed_tf = feedback(L_cur, 1);

figure
margin(L_cur)

grid on

%% 2. 速度环设计 (Middle Loop: i_ref -> dphi_1)
% 使用per-unit 方法进行归一化
T1 = J_1_val;
T2 = J_2_val;
Tc = 1 / k_t_val;

%% 速度环等效对象对比
% 1. 定义基础物理模型
s = tf('s');
num_mech = [J_2_val, 0, k_t_val];
den_mech = [J_1_val*J_2_val, 0, k_t_val*(J_1_val + J_2_val), 0];
sys_mech_dphi1 = tf(num_mech, den_mech);

% 2. 方案 A：裸系统等效对象 (电流环 + 机械模型)
% 这是速度环 PI 之前"看到"的样子
sys_A_equivalent = sys_cur_closed_tf * k_M_val * sys_mech_dphi1;
% sys_A_equivalent = 1 * k_M_val * sys_mech_dphi1;

%% 不引入additional feedback
CTRL.Omega_1.Kp = 2 * sqrt(T1/Tc);
CTRL.Omega_1.Ki = T1 / T2 / Tc ;

%% 引入additional feedback
epsilon_r = 0.7;
CTRL.Omega_1_k1.k1 = 4 * epsilon_r^2 * T1/T2 - 1;
CTRL.Omega_1_k1.Kp = 2 * sqrt(T1 * (1 + CTRL.Omega_1_k1.k1) / Tc);
CTRL.Omega_1_k1.Ki = T1 /T2 / Tc;

epsilon_r = 0.7;
CTRL.Omega_1_k2.k2 = (T2 - 4 * epsilon_r^2 * T1)/(4 * epsilon_r^2 + 1);
CTRL.Omega_1_k2.Kp = 2 * sqrt((T1 + CTRL.Omega_1_k2.k2) * (T2 - CTRL.Omega_1_k2.k2) / T2/Tc);
CTRL.Omega_1_k2.Ki = (T1 + CTRL.Omega_1_k2.k2) / T2 / Tc;

epsilon_r = 0.7;
omega_r_2af = 0.5 * 203;
CTRL.Omega_1_k2pk8.k8 = 1/(omega_r_2af)/T2/Tc - 1;
CTRL.Omega_1_k2pk8.k2 = (T1 + T2) * (1 + CTRL.Omega_1_k2pk8.k8)/...
                        (4 * epsilon_r^2 + 1) - ...
                        T1;
CTRL.Omega_1_k2pk8.Kp = 4 * epsilon_r * omega_r_2af * ...
                        (T1 + CTRL.Omega_1_k2pk8.k2) /...
                        (1 + CTRL.Omega_1_k2pk8.k8);

CTRL.Omega_1_k2pk8.Ki = T2 * Tc * (T1 + CTRL.Omega_1_k2pk8.k2) * omega_r_2af^4;

% CTRL.Omega_1.Kp = 230;
% CTRL.Omega_1.Ki = 1500;
% 
% 
% % 3. 方案 B：带主动阻尼的等效对象 (电流环 + 主动阻尼内环 + 机械模型)
% % 先计算主动阻尼内环 (内环反馈 phi1-phi2)
% J_eq = (J_1_val * J_2_val) / (J_1_val + J_2_val); 
% B_active = 2 * 0.5 * sqrt(k_t_val * J_eq); % 设定 0.5 的目标阻尼比
% 
% % 构造内环反馈项 H_diff (速度差/电机速度)
% H_diff = tf([J_2_val, 0, 0], [J_2_val, 0, k_t_val]); 
% 
% % 得到被内环"驯化"后的机械对象
% sys_mech_damped = feedback(k_M_val * sys_mech_dphi1, B_active * H_diff);
% 
% % 最终方案 B 的等效对象
% sys_B_equivalent = sys_cur_closed_tf * sys_mech_damped;
% 
% % 4. 绘图对比：这才是速度环设计的"起跑线"
% figure('Name', 'Equivalent Plant Comparison');
% bode(sys_A_equivalent, 'r--', sys_B_equivalent, 'b');
% grid on;
% legend('Case A: No Damping (The 130dB Monster)', 'Case B: With Active Damping (Tamed Plant)');
% title('Effective Plant seen by Velocity PI Controller');
% 
% % 5. 打印对比结论
% [mag_A, ~] = bode(sys_A_equivalent, 202.7);
% [mag_B, ~] = bode(sys_B_equivalent, 202.7);
% fprintf('Resonance Peak (Case A): %.2f dB\n', 20*log10(mag_A));
% fprintf('Resonance Peak (Case B): %.2f dB\n', 20*log10(mag_B));
% % 
% % % 2.2 陷波器设计 (此时只需轻轻补一刀)
% % % ========================================================
% % mega_n = 202.7;        
% % zeta_z = 0.05;          % 此时不需要 0.0001 那么深了
% % zeta_p = 0.5;           % 宽度适中，减少相位滞后
% % 
% % num_notch = [1, 2*zeta_z*omega_n, omega_n^2];
% % den_notch = [1, 2*zeta_p*omega_n, omega_n^2];
% % C_notch_s = tf(num_notch, den_notch);
% % 
% % % 2.3 速度 PI 设计
% % % ========================================================
% % CTRL_Vel_Kp = 3219 * 0.018; 
% % CTRL_Vel_Ki = 3219;
% % C_pi = (CTRL_Vel_Kp + CTRL_Vel_Ki/s);
% % 
% % CTRL.Vel.ActiveDamping.B = B_active;
% % CTRL.Vel.ActiveDamping.km = k_M_val;
% % CTRL.Vel.Notch.num = num_notch;
% % CTRL.Vel.Notch.den = den_notch;
% % CTRL.Vel.PID.Kp = CTRL_Vel_Kp;
% % CTRL.Vel.PID.Ki = CTRL_Vel_Ki;
% % 
% % 
% % % 总体开环传递函数
% % L_vel = C_pi * C_notch_s * sys_B_equivalent;
% % 
% % figure('Name', 'Final Design Margin');
% % margin(L_vel); 
% % grid on;
% % 
% % %% 3. 位置环设计 (Outer Loop: dphi_1_ref -> phi_2)
% % % 目标：对消低频极点 (-1.435)，确保负载端位置精度
% % % ========================================================
% % % 提取负载端位置相对于电机速度的关系 (dphi_1 -> phi_2)
% % G_pos_link = sys_mech_tf / sys_iref_to_dphi1; 
% % 
% % % 位置环对象: 速度环闭环 * (dphi_1 -> phi_2)
% % G_pos_obj = Sys_Vel_Closed * G_pos_link;
% % 
% % % 3.1 位置 PD 设计 (对消逻辑: Kp/Kd = 1.435)
% % CTRL.Pos.Kp = 12;
% % CTRL.Pos.Kd = CTRL.Pos.Kp / 1.435; 
% % C_pos = CTRL.Pos.Kp + CTRL.Pos.Kd * s;
% % 
% % % 位置环闭环 (phi_2_ref -> phi_2)
% % L_pos = C_pos * G_pos_obj;
% % Sys_Pos_Closed = feedback(L_pos, 1);
% % 
% % %% 4. 验证与可视化 (含线宽调整)
% % % ========================================================
% % fprintf('--- 控制器参数导出 ---\n');
% % disp(CTRL);
% % 
% % figure('Name', '串级控制系统分析', 'Color', 'w');
% % 
% % % 位置外环开环波特图
% % subplot(2,1,1);
% % h_bode = bodeplot(L_pos);
% % grid on;
% % setoptions(h_bode, 'FreqUnits', 'Hz');
% % title('位置外环开环波特图 (Loop Shaping)');
% % 
% % % 系统闭环阶跃响应
% % subplot(2,1,2);
% % step(Sys_Pos_Closed, 2.0); % 仿真 2 秒
% % grid on;
% % title('系统总闭环阶跃响应 (电压 -> Disc 2 位置)');
% % 
% % % 统一调整当前图窗所有线条宽度
% % set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1.5);
% % 
% % %% 5. 导出标幺值基准
% % % ========================================================
% % CTRL.Base.Voltage = R_val * 1.0; % 假设参考基准
% % CTRL.Base.Position = 180;        % 角度制基准