clear; clc;

%% 1. Symbolic Modelling (Full, Mechanical, and Electrical)
% ========================================================
% Define symbolic variables
% ========================================================
syms R L kE kM J1 J2 kt real
syms U MF MT sc real
syms i phi_1 dphi_1 phi_2 dphi_2

% --- 1.1 Full System (x = [i; phi_1; dphi_1; phi_2; dphi_2]) ---
x = [i; phi_1; dphi_1; phi_2; dphi_2];
A = [ -R/L,      0,      -kE/L,      0,          0;
       0,        0,        1,         0,          0;
      kM/J1,   -kt/J1,     0,       kt/J1,        0;
       0,        0,        0,         0,          1;
       0,      kt/J2,      0,      -kt/J2,        0];
B = [1/L; 0; 0; 0; 0];
E = [0; 0; 0; 0; -1/J2];
C = [0 0 0 1 0]; % Output: phi_2
D = 0;

% --- 1.2 Mechanical Part (xm = [phi_1; dphi_1; phi_2; dphi_2]) ---
xm = [phi_1; dphi_1; phi_2; dphi_2];
Am = [  0,        1,        0,        0;
       -kt/J1,    0,      kt/J1,      0;
        0,        0,        0,        1;
        kt/J2,    0,     -kt/J2,      0];
Bm = [0; 1/J1; 0; 0]; % Input: MT (Motor Torque)
Em = [0; 0; 0; -1/J2]; % Input: MF (Friction)
Cm = [0 0 1 0]; % Output: phi_2
Dm = 0;

% --- 1.3 Electrical Part (xe = [i]) ---
% 注意：在独立建模电气部分用于串级设计时，通常视 dphi_1 为外部扰动耦合
xe = i;
Ae = -R/L;
Be = 1/L;   % Input: U
Ee = -kE/L; % Coupling: dphi_1
Ce = 1;     % Output: i
De = 0;

%% 2. Symbolic Transfer Functions
% ==============================
I5 = eye(size(A));
I4 = eye(size(Am));
I1 = eye(size(Ae));

% Full System: U -> phi_2
G = simplify(C * inv(sc*I5 - A) * B);
% Mechanical: MT -> phi_2
Gm = simplify(Cm * inv(sc*I4 - Am) * Bm);
% Electrical: U -> i
Ge = simplify(Ce * inv(sc*I1 - Ae) * Be);

disp('Full System Transfer Function G(sc) = '); pretty(G)
disp('Mechanical Transfer Function Gm(sc) = '); pretty(Gm)
disp('Electrical Transfer Function Ge(sc) = '); pretty(Ge)

%% 3. Numerical Model Generation
% ==============================
% ß参数脚本
prepare_parameter

% --- 3.1 Numerical Full System (U -> phi_2) ---
A_num = double(subs(A, [J1 J2 kt kE R kM L], [J_1_val J_2_val k_t_val k_E_val R_val k_M_val L_val]));
B_num = double(subs(B, [L], [L_val]));
E_num = double(subs(E, [J2], [J_2_val]));
C_num = [0 0 0 1 0];
D_num = 0;

sys_full_ss  = ss(A_num, B_num, C_num, D_num);
sys_full_tf  = tf(sys_full_ss);
sys_full_zpk = zpk(sys_full_ss);

% --- 3.2 Numerical Full System Degree (U -> phi_2_deg) ---
C_num_to_degree = C_num * 180 / pi;
sys_full_degree_ss  = ss(A_num, B_num, C_num_to_degree, D_num);
sys_full_degree_tf  = tf(sys_full_degree_ss);
sys_full_degree_zpk = zpk(sys_full_degree_ss);

% --- 3.3 Numerical Mechanical Part (MT -> phi_2) ---
Am_num = double(subs(Am, [J1 J2 kt], [J_1_val J_2_val k_t_val]));
Bm_num = double(subs(Bm, [J1 J2], [J_1_val J_2_val]));
Em_num = double(subs(Em, [J1 J2], [J_1_val J_2_val]));
Cm_num = [0 0 1 0];
Dm_num = 0;

sys_mech_ss  = ss(Am_num, Bm_num, Cm_num, Dm_num);
sys_mech_tf  = tf(sys_mech_ss);
sys_mech_zpk = zpk(sys_mech_ss);

% --- 3.4 Numerical Electrical Part (U -> i) ---
Ae_num = double(subs(Ae, [R L], [R_val L_val]));
Be_num = double(subs(Be, [L], [L_val]));
Ce_num = 1;
De_num = 0;

sys_elec_ss  = ss(Ae_num, Be_num, Ce_num, De_num);
sys_elec_tf  = tf(sys_elec_ss);
sys_elec_zpk = zpk(sys_elec_ss);

%% 4. Model Verification and Display
% ==================================
fprintf('\n================================================\n');
fprintf('Model Summary:\n');
fprintf('Electrical Pole: %.2f rad/s\n', abs(eigs(Ae_num)));
fprintf('Mechanical Resonance: %.2f rad/s\n', sqrt(k_t_val*(J_1_val+J_2_val)/(J_1_val*J_2_val))); % 计算双质量谐振频率
fprintf('================================================\n');

disp('Electrical System (ZPK):'); disp(sys_elec_zpk)
disp('Mechanical System (ZPK):'); disp(sys_mech_zpk)
disp('Full System (ZPK):');       disp(sys_full_zpk)

% 绘制波特图对比
figure('Name', 'System Frequency Response Comparison', 'Color', 'w');
bode(sys_full_tf, 'b', sys_mech_tf, 'r--', sys_elec_tf, 'm:');
grid on;
legend('Full System (U \rightarrow \phi_2)', 'Mech Part (MT \rightarrow \phi_2)', 'Elec Part (U \rightarrow i)');