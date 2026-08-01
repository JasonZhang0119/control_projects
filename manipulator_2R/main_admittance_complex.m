%% main_admittance_control_fancy_demo.m
% Fancy 2R end-effector admittance control demo for PPT
%
% Demo idea
% ---------
% 1. End-effector moves along a planned free-space path.
% 2. It touches a virtual vertical wall.
% 3. Admittance controller modifies the desired end-effector position.
% 4. Robot yields in the wall-normal direction while sliding tangentially.
%
% Control chain
% -------------
% x_nominal trajectory + Fext
%       -> EndAdmittanceControl
%       -> xd_cmd
%       -> IK2R_Continuous
%       -> JointPositionPIDController
%       -> Robot dynamics + virtual environment

clear;
clc;
close all;

%% ================================================================
% 1. Path configuration
% ================================================================
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root_dir, 'matlab')));

%% ================================================================
% 2. Robot parameters
% ================================================================
param = Param2R('standard');
n = numel(param.joint_types);

%% ================================================================
% 3. Simulation parameters
% ================================================================
Ts = 0.001;
T_total = 8.0;
N = round(T_total / Ts);
t = (0:N-1) * Ts;

%% ================================================================
% 4. Initial state
% ================================================================
q0 = deg2rad([35; -70]);
x0 = FK2R_Analytic(q0, param);

q = zeros(n, N);
qd = zeros(n, N);
qdd = zeros(n, N);

q(:, 1) = q0;
qd(:, 1) = zeros(n, 1);
qdd(:, 1) = zeros(n, 1);

%% ================================================================
% 5. Virtual wall parameters
% ================================================================
% vertical wall: x = x_wall
x_wall = x0(1) + 0.10;

K_env = 1800;
D_env = 35;

%% ================================================================
% 6. Fancy nominal trajectory
% ================================================================
% The nominal path tries to pass through the wall.
% The admittance controller should make the actual command retreat.
x_start = x0;
x_goal  = x0 + [0.18; 0.12];

s = smoothstep(t / T_total);
x_nom_log = zeros(2, N);
xdot_nom_log = zeros(2, N);
xddot_nom_log = zeros(2, N);

for k = 1:N
    % Curved path with a small sinusoidal y modulation
    x_nom_log(:, k) = x_start + s(k) * (x_goal - x_start);
    x_nom_log(2, k) = x_nom_log(2, k) + 0.025 * sin(2 * pi * s(k));

    if k > 1
        xdot_nom_log(:, k) = (x_nom_log(:, k) - x_nom_log(:, k-1)) / Ts;
    end
end

for k = 2:N
    xddot_nom_log(:, k) = (xdot_nom_log(:, k) - xdot_nom_log(:, k-1)) / Ts;
end

%% ================================================================
% 7. Admittance controller parameters
% ================================================================
adm_param.Md = diag([2.0, 1.5]);
adm_param.Dd = diag([45.0, 25.0]);
adm_param.Kd = diag([120.0, 50.0]);

adm_options = struct();
adm_options.task_dim = 2;
adm_options.x0 = x_start;
adm_options.xdot0 = zeros(2, 1);
adm_options.xddot0 = zeros(2, 1);

adm_controller = EndAdmittanceControl(adm_param, param, Ts, adm_options);

%% ================================================================
% 8. Inner joint position controller
% ================================================================
pos_pid_param.Kp = [220.0; 180.0];
pos_pid_param.Ki = [0.0; 0.0];
pos_pid_param.Kd = [18.0; 16.0];

pos_options = struct();
pos_options.tau_min = -100 * ones(n, 1);
pos_options.tau_max =  100 * ones(n, 1);

pos_controller = JointPositionPIDController(pos_pid_param, param, Ts, pos_options);

%% ================================================================
% 9. Logs
% ================================================================
x_log = zeros(2, N);
xdot_log = zeros(2, N);
xddot_log = zeros(2, N);

xd_cmd_log = zeros(2, N);
xdotd_cmd_log = zeros(2, N);
xddotd_cmd_log = zeros(2, N);

Fext_log = zeros(2, N);
q_ref_log = zeros(n, N);
tau_pos_log = zeros(n, N);
tau_env_log = zeros(n, N);
tau_total_log = zeros(n, N);

penetration_log = zeros(1, N);

%% ================================================================
% 10. Main loop
% ================================================================
xdot_prev = zeros(2, 1);
q_ref_prev = q0;

xd_cmd = x_start;
xdotd_cmd = zeros(2, 1);
xddotd_cmd = zeros(2, 1);

for k = 1:N-1

    % ------------------------------------------------------------
    % Current end-effector state
    % ------------------------------------------------------------
    x = FK2R_Analytic(q(:, k), param);
    J = Jacobian2R_Analytic(q(:, k), param);
    xdot = J * qd(:, k);

    if k == 1
        xddot = zeros(2, 1);
    else
        xddot = (xdot - xdot_prev) / Ts;
    end
    xdot_prev = xdot;

    x_log(:, k) = x;
    xdot_log(:, k) = xdot;
    xddot_log(:, k) = xddot;

    % ------------------------------------------------------------
    % Virtual wall contact model
    % ------------------------------------------------------------
    penetration = x(1) - x_wall;
    Fext_on_robot = zeros(2, 1);

    if penetration > 0
        Fext_on_robot(1) = -K_env * penetration - D_env * xdot(1);

        % Small tangential friction-like force for visual realism
        Fext_on_robot(2) = -2.0 * xdot(2);
    end

    penetration_log(k) = max(penetration, 0);
    Fext_log(:, k) = Fext_on_robot;

    % ------------------------------------------------------------
    % Admittance control
    % ------------------------------------------------------------
    % 推荐你的 EndAdmittanceControl 内部使用：
    %
    % Md * xddotd + Dd * (xdotd - xdot_nom) + Kd * (xd - x_nom) = Fext
    %
    % 如果你当前版本只支持以固定 x0 为平衡点，则先可以直接传 Fext。
    [xd_next, info_adm] = adm_controller.step( ...
        Fext_on_robot, ...
        x, ...
        xdot, ...
        xddot);

    % Add nominal path bias for PPT demo
    % This line makes the command follow the nominal path while admittance
    % introduces yielding behavior.
    alpha_nom = 0.015;
    xd_cmd = xd_next + alpha_nom * (x_nom_log(:, k) - xd_next);

    xdotd_cmd = info_adm.xdotd;
    xddotd_cmd = info_adm.xddotd;

    xd_cmd_log(:, k) = xd_cmd;
    xdotd_cmd_log(:, k) = xdotd_cmd;
    xddotd_cmd_log(:, k) = xddotd_cmd;

    % ------------------------------------------------------------
    % Inverse kinematics
    % ------------------------------------------------------------
    try
        q_ref = IK2R_Continuous(xd_cmd, param, q_ref_prev);
    catch ME
        warning('IK failed. Keep previous q_ref: %s', ME.message);
        q_ref = q_ref_prev;
    end

    q_ref_prev = q_ref;
    q_ref_log(:, k) = q_ref;

    % ------------------------------------------------------------
    % Joint position control
    % ------------------------------------------------------------
    qd_ref = zeros(n, 1);
    qdd_ref = zeros(n, 1);

    tau_pos = pos_controller.step(q_ref, qd_ref, qdd_ref, q(:, k), qd(:, k));
    tau_pos_log(:, k) = tau_pos;

    % ------------------------------------------------------------
    % Environment force mapping
    % ------------------------------------------------------------
    tau_env = J' * Fext_on_robot;
    tau_env_log(:, k) = tau_env;

    % ------------------------------------------------------------
    % Robot dynamics
    % ------------------------------------------------------------
    [M, h, G] = Robot2R_Dynamics(q(:, k), qd(:, k), param);

    tau_total = tau_pos + tau_env;
    tau_total_log(:, k) = tau_total;

    qdd_current = M \ (tau_total - h - G);

    qd(:, k+1) = qd(:, k) + qdd_current * Ts;
    q(:, k+1)  = q(:, k)  + qd(:, k+1) * Ts;
    qdd(:, k+1) = qdd_current;
end

%% ================================================================
% 11. Fill last sample
% ================================================================
x_log(:, N) = FK2R_Analytic(q(:, N), param);
J = Jacobian2R_Analytic(q(:, N), param);
xdot_log(:, N) = J * qd(:, N);

xd_cmd_log(:, N) = xd_cmd;
xdotd_cmd_log(:, N) = xdotd_cmd;
xddotd_cmd_log(:, N) = xddotd_cmd;

q_ref_log(:, N) = q_ref_prev;
Fext_log(:, N) = Fext_log(:, N-1);
tau_pos_log(:, N) = tau_pos_log(:, N-1);
tau_env_log(:, N) = tau_env_log(:, N-1);
tau_total_log(:, N) = tau_total_log(:, N-1);
penetration_log(N) = penetration_log(N-1);

%% ================================================================
% 12. PPT-friendly plots
% ================================================================
figure('Name', 'Fancy Admittance Control Trajectory', 'Color', 'w');
set(gcf, 'Position', [100, 100, 1000, 680]);

plot(x_nom_log(1, :), x_nom_log(2, :), '--', 'LineWidth', 1.5);
hold on;
plot(xd_cmd_log(1, :), xd_cmd_log(2, :), 'LineWidth', 2.0);
plot(x_log(1, :), x_log(2, :), 'LineWidth', 2.0);

y_min = min([x_nom_log(2, :), x_log(2, :), xd_cmd_log(2, :)]) - 0.05;
y_max = max([x_nom_log(2, :), x_log(2, :), xd_cmd_log(2, :)]) + 0.05;

plot([x_wall, x_wall], [y_min, y_max], ':', 'LineWidth', 2.5);

scatter(x_log(1, 1), x_log(2, 1), 80, 'filled');
scatter(x_log(1, end), x_log(2, end), 80, 'filled');

axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('End-effector Admittance Response near Virtual Wall');
legend('Nominal path', 'Admittance command', 'Actual path', ...
       'Virtual wall', 'Start', 'End', ...
       'Location', 'best');

%% Force and penetration
figure('Name', 'Contact Force and Penetration', 'Color', 'w');
set(gcf, 'Position', [150, 150, 1000, 560]);

subplot(2, 1, 1);
plot(t, Fext_log(1, :), 'LineWidth', 1.5);
hold on;
plot(t, Fext_log(2, :), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Force [N]');
title('External Contact Force');
legend('F_x', 'F_y');

subplot(2, 1, 2);
plot(t, penetration_log * 1000, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Penetration [mm]');
title('Virtual Wall Penetration');

%% Position comparison
figure('Name', 'Position Tracking Detail', 'Color', 'w');
set(gcf, 'Position', [200, 200, 1000, 620]);

subplot(2, 1, 1);
plot(t, x_nom_log(1, :), '--', 'LineWidth', 1.3);
hold on;
plot(t, xd_cmd_log(1, :), 'LineWidth', 1.5);
plot(t, x_log(1, :), 'LineWidth', 1.5);
yline(x_wall, ':', 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('x [m]');
title('Wall-normal Direction');
legend('x_{nom}', 'x_d', 'x', 'wall');

subplot(2, 1, 2);
plot(t, x_nom_log(2, :), '--', 'LineWidth', 1.3);
hold on;
plot(t, xd_cmd_log(2, :), 'LineWidth', 1.5);
plot(t, x_log(2, :), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('y [m]');
title('Wall-tangential Direction');
legend('y_{nom}', 'y_d', 'y');

%% Joint response
figure('Name', 'Joint Response', 'Color', 'w');
set(gcf, 'Position', [250, 250, 1000, 560]);

subplot(2, 1, 1);
plot(t, rad2deg(q(1, :)), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(q(2, :)), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint angle [deg]');
title('Joint Angles');
legend('q_1', 'q_2');

subplot(2, 1, 2);
plot(t, tau_total_log(1, :), 'LineWidth', 1.5);
hold on;
plot(t, tau_total_log(2, :), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Total Joint Torque');
legend('\tau_1', '\tau_2');

%% ================================================================
% 13. Optional animation
% ================================================================
make_animation = true;

if make_animation
    figure('Name', '2R Robot Fancy Animation', 'Color', 'w');
    set(gcf, 'Position', [300, 120, 900, 720]);

    skip = 25;

    for k = 1:skip:N
        clf;

        qk = q(:, k);
        p0 = [0; 0];

        l1 = param.l1;
        l2 = param.l1;

        p1 = [l1 * cos(qk(1));
              l1 * sin(qk(1))];

        p2 = p1 + [l2 * cos(qk(1) + qk(2));
                   l2 * sin(qk(1) + qk(2))];

        plot([p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], ...
             '-o', 'LineWidth', 3.0, 'MarkerSize', 7);
        hold on;

        plot(x_nom_log(1, 1:k), x_nom_log(2, 1:k), '--', 'LineWidth', 1.2);
        plot(xd_cmd_log(1, 1:k), xd_cmd_log(2, 1:k), 'LineWidth', 1.5);
        plot(x_log(1, 1:k), x_log(2, 1:k), 'LineWidth', 2.0);

        plot([x_wall, x_wall], [-0.35, 0.35], ':', 'LineWidth', 2.5);

        if Fext_log(1, k) ~= 0
            quiver(p2(1), p2(2), 0.0008 * Fext_log(1, k), 0.0008 * Fext_log(2, k), ...
                   'LineWidth', 2.0, 'MaxHeadSize', 2.0);
        end

        axis equal;
        grid on;
        xlim([-0.05, 0.45]);
        ylim([-0.25, 0.35]);

        xlabel('x [m]');
        ylabel('y [m]');
        title(sprintf('Admittance Control Demo, t = %.2f s', t(k)));

        legend('Robot', 'Nominal path', 'Admittance command', ...
               'Actual path', 'Virtual wall', 'Contact force', ...
               'Location', 'bestoutside');

        drawnow;
    end
end

%% ================================================================
% Local function
% ================================================================
function y = smoothstep(x)
%SMOOTHSTEP Smooth transition from 0 to 1.
%
% Parameters
% ----------
% x : double array
%     Normalized time variable.
%
% Returns
% -------
% y : double array
%     Smoothed scalar profile.

    x = min(max(x, 0), 1);
    y = 3 * x.^2 - 2 * x.^3;
end