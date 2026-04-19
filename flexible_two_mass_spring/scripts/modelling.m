
clear
clc

%% Symbolic state-space model of a two-mass rotor system

%% Define symbolic variables
syms R L kE kM J1 J2 kt real
syms U MF real

% Full-state vector
syms i phi_1 dphi_1 phi_2 dphi_2

x = [i;
     phi_1;
     dphi_1;
     phi_2;
     dphi_2];

%% Full electromechanical model matrices
A = [ -R/L,      0,      -kE/L,      0,          0;
       0,        0,        1,         0,          0;
      kM/J1,   -kt/J1,     0,       kt/J1,        0;
       0,        0,        0,         0,          1;
       0,      kt/J2,      0,      -kt/J2,        0];

% Input matrix for applied voltage U
B = [1/L;
     0;
     0;
     0;
     0];

% Disturbance matrix for friction torque MF
E = [0;
     0;
     0;
     0;
    -1/J2];

% Output matrix for measured angle phi_2
C = [0 0 0 1 0];

% Symbolic state equation
xdot = A*x + B*U + E*MF;

%% Display full-model symbolic forms
disp('State matrix A = ')
pretty(A)

disp('Input matrix B = ')
pretty(B)

disp('Friction matrix E = ')
pretty(E)

disp('State equation xdot = ')
pretty(xdot)

%% Mechanical-only model (motor torque as input)

syms MT

xm = [phi_1;
      dphi_1;
      phi_2;
      dphi_2];

% Mechanical state matrix
Am = [  0,        1,        0,        0;
       -kt/J1,    0,      kt/J1,      0;
        0,        0,        0,        1;
        kt/J2,    0,     -kt/J2,      0];

% Input matrix for motor torque MT
Bm = [  0;
       1/J1;
        0;
        0];

% Disturbance matrix for friction MF
Em = [  0;
        0;
        0;
      -1/J2];

% State equation
xmdot = Am*xm + Bm*MT + Em*MF;

% Output equation (phi_2)
Cm = [0 0 1 0];
ym = Cm*xm;

%% Display mechanical-model symbolic forms
disp('Mechanical State Matrix Am = ')
pretty(Am)

disp('Input matrix Bm (MT) = ')
pretty(Bm)

disp('Friction matrix Em (MF) = ')
pretty(Em)

disp('State equation xmdot = ')
pretty(xmdot)

disp('Output equation y = ')
pretty(ym)

%% Symbolic transfer functions

syms sc

% Mechanical transfer function Gm(sc) = phi_2 / MT
Im = eye(size(Am));
Gm = simplify(Cm * inv(sc*Im - Am) * Bm);

disp('Transfer Function Gm(sc) = ')
pretty(Gm)

% Full electromechanical transfer function G(sc) = phi_2 / U
I = eye(size(A));
G = simplify(C * inv(sc*I - A) * B);

disp('Transfer Function G(sc) = ')
pretty(G)

%% Load numeric parameters
prepare_parameters

%% Numeric mechanical model
Am_num = double(subs(Am, ...
    [J1 J2 kt], ...
    [J_1_val J_2_val k_t_val]));

Bm_num = double(subs(Bm, [J1 J2], [J_1_val J_2_val]));
Em_num = double(subs(Em, [J1 J2], [J_1_val J_2_val]));

Cm_num = [0 0 1 0];
Dm_num = 0;

sys_mech_ss = ss(Am_num, Bm_num, Cm_num, Dm_num);

eigs(Am_num)

sys_mech_tf = tf(sys_mech_ss);
zpk(sys_mech_tf)

%% Numeric full electromechanical model
A_num = double(subs(A, ...
    [J1 J2 kt kE R kM L], ...
    [J_1_val J_2_val k_t_val k_E_val, R_val, k_M_val, L_val]));

B_num = double(subs(B, [L], [L_val]));
E_num = double(subs(E, [J2], [J_2_val]));

C_num = [0 0 0 1 0];
D_num = 0;

sys_full_ss = ss(A_num, B_num, C_num, D_num);

eigs(A_num);

sys_full_tf = tf(sys_full_ss);
sys_full_zpk = zpk(sys_full_tf);

%% Convert output from rad to deg for controller design
C_num_to_degree = C_num * 180 / pi;

sys_full_degree_ss = ss(A_num, B_num, C_num_to_degree, D_num);
sys_full_degree_tf = tf(sys_full_degree_ss);

%% Quick open-loop margin check (degree output model)
figure
margin(sys_full_degree_tf)

%% Discrete-time step used in companion simulations
Ts = 0.0001;
