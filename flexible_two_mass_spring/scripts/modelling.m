clear; clc; close all;

%% 1. Symbolic Modelling (Full, Mechanical, and Electrical)
% ========================================================
% Define symbolic variables
% ========================================================
syms R L kE kM J1 J2 kt real
syms U MF MT sc real
syms i phi_1 dphi_1 phi_2 dphi_2 real

% --- 1.1 Full System (x = [i; phi_1; dphi_1; phi_2; dphi_2]) ---
x = [i; phi_1; dphi_1; phi_2; dphi_2];
A = [ -R/L,      0,      -kE/L,      0,          0;
       0,        0,        1,         0,          0;
      kM/J1,   -kt/J1,     0,       kt/J1,        0;
       0,        0,        0,         0,          1;
       0,      kt/J2,      0,      -kt/J2,        0];
B = [1/L; 0; 0; 0; 0];
E = [0; 0; 0; 0; -1/J2];

% 原输出：phi_2
C = [0 0 0 1 0];
D = 0;

% --- 1.2 Mechanical Part (xm = [phi_1; dphi_1; phi_2; dphi_2]) ---
xm = [phi_1; dphi_1; phi_2; dphi_2];
Am = [  0,        1,        0,        0;
       -kt/J1,    0,      kt/J1,      0;
        0,        0,        0,        1;
        kt/J2,    0,     -kt/J2,      0];
Bm = [0; 1/J1; 0; 0];        % Input: MT (Motor Torque)
Em = [0; 0; 0; -1/J2];       % Input: MF (Friction)

% Mechanical outputs
Cm_phi2   = [0 0 1 0];       % phi_2
Cm_omega1 = [0 1 0 0];       % dphi_1 = omega_1
Cm_omega2 = [0 0 0 1];       % dphi_2 = omega_2
Dm = 0;

% --- 1.3 Electrical Part (xe = [i]) ---
% 注意：在独立建模电气部分用于串级设计时，通常视 dphi_1 为外部扰动耦合
xe = i;
Ae = -R/L;
Be = 1/L;    % Input: U
Ee = -kE/L;  % Coupling: dphi_1
Ce = 1;      % Output: i
De = 0;

%% 2. Symbolic Transfer Functions
% ==============================
I5 = eye(size(A));
I4 = eye(size(Am));
I1 = eye(size(Ae));

% Full System: U -> phi_2
G = simplify(C * inv(sc * I5 - A) * B);

% Mechanical: MT -> phi_2
Gm_phi2 = simplify(Cm_phi2 * inv(sc * I4 - Am) * Bm);

% Mechanical: MT -> omega_1 (用于观察反共振)
Gm_omega1 = simplify(Cm_omega1 * inv(sc * I4 - Am) * Bm);

% Mechanical: MT -> omega_2 (用于对比)
Gm_omega2 = simplify(Cm_omega2 * inv(sc * I4 - Am) * Bm);

% Electrical: U -> i
Ge = simplify(Ce * inv(sc * I1 - Ae) * Be);

disp('Full System Transfer Function G(sc) = U -> phi_2'); 
pretty(G)

disp('Mechanical Transfer Function Gm_phi2(sc) = MT -> phi_2'); 
pretty(Gm_phi2)

disp('Mechanical Transfer Function Gm_omega1(sc) = MT -> omega_1'); 
pretty(Gm_omega1)

disp('Mechanical Transfer Function Gm_omega2(sc) = MT -> omega_2'); 
pretty(Gm_omega2)

disp('Electrical Transfer Function Ge(sc) = U -> i'); 
pretty(Ge)

%% 3. Numerical Model Generation
% ==============================
% 参数脚本
prepare_parameters

% --- 3.1 Numerical Full System (U -> phi_2) ---
A_num = double(subs(A, [J1 J2 kt kE R kM L], ...
    [J_1_val J_2_val k_t_val k_E_val R_val k_M_val L_val]));
B_num = double(subs(B, [L], [L_val]));
E_num = double(subs(E, [J2], [J_2_val]));
C_num = [0 0 0 1 0];
D_num = 0;

sys_full_ss  = ss(A_num, B_num, C_num, D_num);
sys_full_tf  = minreal(tf(sys_full_ss));
sys_full_zpk = zpk(sys_full_ss);

% --- 3.2 Numerical Full System Degree (U -> phi_2_deg) ---
C_num_to_degree = C_num * 180 / pi;
sys_full_degree_ss  = ss(A_num, B_num, C_num_to_degree, D_num);
sys_full_degree_tf  = minreal(tf(sys_full_degree_ss));
sys_full_degree_zpk = zpk(sys_full_degree_ss);

% --- 3.3 Numerical Mechanical Part (MT -> phi_2) ---
Am_num = double(subs(Am, [J1 J2 kt], [J_1_val J_2_val k_t_val]));
Bm_num = double(subs(Bm, [J1], [J_1_val]));
Em_num = double(subs(Em, [J2], [J_2_val]));

Cm_phi2_num   = [0 0 1 0];
Cm_omega1_num = [0 1 0 0];
Cm_omega2_num = [0 0 0 1];
Dm_num = 0;

sys_mech_phi2_ss  = ss(Am_num, Bm_num, Cm_phi2_num, Dm_num);
sys_mech_phi2_tf  = minreal(tf(sys_mech_phi2_ss));
sys_mech_phi2_zpk = zpk(sys_mech_phi2_ss);

% --- 3.4 Mechanical model for anti-resonance: MT -> omega_1 ---
sys_mech_omega1_ss  = ss(Am_num, Bm_num, Cm_omega1_num, Dm_num);
sys_mech_omega1_tf  = minreal(tf(sys_mech_omega1_ss));
sys_mech_omega1_zpk = zpk(sys_mech_omega1_ss);

% --- 3.5 Mechanical comparison model: MT -> omega_2 ---
sys_mech_omega2_ss  = ss(Am_num, Bm_num, Cm_omega2_num, Dm_num);
sys_mech_omega2_tf  = minreal(tf(sys_mech_omega2_ss));
sys_mech_omega2_zpk = zpk(sys_mech_omega2_ss);

% --- 3.6 Numerical Electrical Part (U -> i) ---
Ae_num = double(subs(Ae, [R L], [R_val L_val]));
Be_num = double(subs(Be, [L], [L_val]));
Ce_num = 1;
De_num = 0;

sys_elec_ss  = ss(Ae_num, Be_num, Ce_num, De_num);
sys_elec_tf  = minreal(tf(sys_elec_ss));
sys_elec_zpk = zpk(sys_elec_ss);

%% 4. Model Verification and Display
% ==================================
omega_res = sqrt(k_t_val * (J_1_val + J_2_val) / (J_1_val * J_2_val));
omega_ar  = sqrt(k_t_val / J_2_val);

fprintf('\n================================================\n');
fprintf('Model Summary:\n');
fprintf('Electrical Pole Magnitude        : %.4f rad/s\n', abs(eigs(Ae_num)));
fprintf('Mechanical Resonance Frequency   : %.4f rad/s\n', omega_res);
fprintf('Mechanical Anti-Resonance Theory : %.4f rad/s\n', omega_ar);
fprintf('================================================\n');

disp('Electrical System (ZPK):');
disp(sys_elec_zpk)

disp('Mechanical System MT -> phi_2 (ZPK):');
disp(sys_mech_phi2_zpk)

disp('Mechanical System MT -> omega_1 (ZPK):');
disp(sys_mech_omega1_zpk)

disp('Mechanical System MT -> omega_2 (ZPK):');
disp(sys_mech_omega2_zpk)

disp('Full System U -> phi_2 (ZPK):');
disp(sys_full_zpk)

%% 5. Frequency-Domain Plots
% ==========================
% 5.1 原始对比图
figure('Name', 'System Frequency Response Comparison', 'Color', 'w');
bode(sys_full_tf, 'b', sys_mech_phi2_tf, 'r--', sys_elec_tf, 'm:');
grid on;
legend('Full System (U -> \phi_2)', ...
       'Mech Part (MT -> \phi_2)', ...
       'Elec Part (U -> i)', ...
       'Location', 'best');

% 5.2 反共振观察图：重点看 omega_1 通道
figure('Name', 'Mechanical Anti-Resonance Observation', 'Color', 'w');
bode(sys_mech_omega1_tf, 'b', sys_mech_omega2_tf, 'r--');
grid on;
legend('MT -> \omega_1  (motor-side, expect anti-resonance)', ...
       'MT -> \omega_2  (load-side, no finite anti-resonance)', ...
       'Location', 'best');

% 5.3 零极点图，直接看 MT -> omega_1 的零点
figure('Name', 'Pole-Zero Map for MT -> omega_1', 'Color', 'w');
pzmap(sys_mech_omega1_tf);
grid on;
title('Pole-Zero Map of Mechanical Channel: MT -> \omega_1');

%% 6. Optional: Evaluate response near theoretical anti-resonance frequency
% =========================================================================
[mag1, phase1, wout1] = bode(sys_mech_omega1_tf, omega_ar);
mag1 = squeeze(mag1);
phase1 = squeeze(phase1);

fprintf('\nAt theoretical anti-resonance frequency omega_ar = %.4f rad/s:\n', omega_ar);
fprintf('Magnitude of MT -> omega_1 : %.6e\n', mag1);
fprintf('Phase of MT -> omega_1     : %.4f deg\n', phase1);