
%% Physical and actuator parameters
% Mass [kg]
m_1_val = 5;
m_2_val = 2;

% Radii [m]
r_1_val = 250 * 1e-3;
r_2_val = 80 * 1e-3;
r_s_val = 5 * 1e-3;

% Shaft length [m]
l_val = 300 * 1e-3;

% Shear modulus [Pa]
G_val = 77.2 * 1e9;

% Electrical parameters
R_val = 0.32;          % Resistance [Ohm]
L_val = 1.04 * 1e-3;   % Inductance [H]
k_E_val = 0.2938;      % Back-EMF constant
k_M_val = 0.253;       % Torque constant
U_max_val = 48;        % Voltage saturation [V]
I_max_val = 20;        % Current saturation [A]

% Friction torque [N*m]
M_F_val = 0.5;

%% Derived mechanical parameters
% Disk inertias [kg*m^2]
J_1_val = 1/2 * m_1_val * r_1_val^2;
J_2_val = 1/2 * m_2_val * r_2_val^2;

% Torsional stiffness [N*m/rad]
k_t_val = G_val * pi * r_s_val^4 / 2 / l_val;

%% Simulation sample time [s]
Ts = 1e-4;
