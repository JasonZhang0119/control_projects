% MATLAB Script: Phase Portrait Analysis with Coulomb Friction
clear; clc; close all;

% --- 1. System Parameters ---
J1 = 0.15625; J2 = 0.0064; kt = 252.64;
R = 0.32; L = 0.00104; kE = 0.2938; kM = 0.253;
MF = 0.5;
kp = 0.02; kd = 0.5/360; 
phi_ref_deg = 200; 
phi_ref = phi_ref_deg * pi / 180; % Convert to radians for physics engine
U_max = 48; I_max = 20;

% Calculate theoretical deadzone in degrees
deadzone_deg = (MF * R) / (kM * kp);

% --- 2. Simulation Setup ---
tspan = [0 5];

% Define multiple initial conditions to observe trajectory convergence
% Format: [phi1, phi1_dot, phi2, phi2_dot, I]
initial_conditions = [
    0, 0, 0, 0, 0;  % Starting from 0 as requested in task 2
    phi_ref + 0.8, 0, phi_ref + 0.8, 0, 0; % Starting slightly ahead
    phi_ref - 1.2, 0, phi_ref - 1.2, 0, 0  % Starting slightly behind
];

% Initialize Plot
figure('Name', 'Phase Portrait of Disc 2', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on;

% Plot the Equilibrium Segment (Deadzone) on the horizontal axis
plot([-deadzone_deg, deadzone_deg], [0, 0], 'r-', 'LineWidth', 5, ...
    'DisplayName', 'Equilibrium Segment (Deadband)');

% --- 3. Run Simulations ---
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :)';
    
    % ode15s is better suited for the "stiff" behavior caused by stiction
    [t, x] = ode15s(@(t, x) rotor_sys(t, x, phi_ref, J1, J2, kt, R, L, kE, kM, MF, kp, kd, U_max), tspan, x0);
    
    % Extract error e (degrees) and e_dot (degrees/s)
    e_deg = (x(:, 3) - phi_ref) * 180 / pi;
    edot_deg = x(:, 4) * 180 / pi;
    
    % Plot trajectory
    plot(e_deg, edot_deg, 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Mark start (circle) and end (cross) points
    plot(e_deg(1), edot_deg(1), 'bo', 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
    plot(e_deg(end), edot_deg(end), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
end

% Formatting the plot
xlabel('Angle Error $e = \varphi_2 - \varphi_{Ref}$ (deg)', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Angular Velocity $\dot{\varphi}_2$ (deg/s)', 'Interpreter', 'latex', 'FontSize', 12);
title('Phase Portrait: System Trapped in Deadzone', 'Interpreter', 'latex', 'FontSize', 14);
legend('Location', 'northeast', 'FontSize', 11);
xlim([-40 40]);
ylim([-200 200]);

% --- 4. ODE Function Definition ---
function dxdt = rotor_sys(t, x, phi_ref, J1, J2, kt, R, L, kE, kM, MF, kp, kd, U_max)
    phi1 = x(1); phi1_dot = x(2);
    phi2 = x(3); phi2_dot = x(4);
    I = x(5);
    
    % Calculate error in degrees (as controller parameters are tuned for deg)
    e_deg = (phi_ref - phi2) * 180 / pi;
    edot_deg = (0 - phi2_dot) * 180 / pi;
    
    % PD Control Law with voltage saturation
    U_raw = kp * e_deg + kd * edot_deg; 
    U = max(min(U_raw, U_max), -U_max);
    
    % Friction model: Tanh approximation of sign() to prevent ODE solver from stalling
    % gamma controls the steepness of the friction transition
    gamma = 50; 
    friction_torque = MF * tanh(gamma * phi2_dot);
    
    % State derivatives matrix calculation
    dxdt = zeros(5,1);
    dxdt(1) = phi1_dot;
    dxdt(2) = (kM * I - kt * (phi1 - phi2)) / J1;
    dxdt(3) = phi2_dot;
    dxdt(4) = (kt * (phi1 - phi2) - friction_torque) / J2;
    dxdt(5) = (U - R * I - kE * phi1_dot) / L;
end