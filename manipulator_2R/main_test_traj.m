clear; clc; close all;

%% Trajectory waypoints
q_waypoints = [
    0.00,  0.35,  0.70,  1.05;
    0.25,  0.05, -0.20, -0.45
];
t_waypoints = [0.0, 1.5, 3.0, 4.5];

%% Build trajectory
traj = CreateCubicSpline(q_waypoints, t_waypoints);

%% Evaluate trajectory
N = 401;
t_vec = linspace(t_waypoints(1), t_waypoints(end), N);
n = size(q_waypoints, 1);

q_ref = zeros(n, N);
q_dot_ref = zeros(n, N);
q_ddot_ref = zeros(n, N);

for k = 1:N
    t = t_vec(k);
    [q_ref(:, k), q_dot_ref(:, k), q_ddot_ref(:, k)] = EvalMultiSegmentCubic(traj, t);
end

%% Compact 3-panel plot
figure('Name', 'Interpolation Curve Summary');

subplot(3, 1, 1); hold on;
plot(t_vec, q_ref(1, :), 'LineWidth', 1.8);
plot(t_vec, q_ref(2, :), 'LineWidth', 1.8);
plot(t_waypoints, q_waypoints(1, :), 'ko', 'MarkerSize', 6, 'LineWidth', 1.2);
plot(t_waypoints, q_waypoints(2, :), 'ks', 'MarkerSize', 6, 'LineWidth', 1.2);
grid on;
ylabel('q');
title('Position');
legend('q_1', 'q_2', 'q_1 waypoints', 'q_2 waypoints', 'Location', 'best');

subplot(3, 1, 2); hold on;
plot(t_vec, q_dot_ref(1, :), 'LineWidth', 1.8);
plot(t_vec, q_dot_ref(2, :), 'LineWidth', 1.8);
grid on;
ylabel('q dot');
title('Velocity');
legend('q dot_1', 'q dot_2', 'Location', 'best');

subplot(3, 1, 3); hold on;
plot(t_vec, q_ddot_ref(1, :), 'LineWidth', 1.8);
plot(t_vec, q_ddot_ref(2, :), 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('q ddot');
title('Acceleration');
legend('q ddot_1', 'q ddot_2', 'Location', 'best');

%% Waypoint connection plot
figure('Name', 'Waypoint Connection');
plot(q_waypoints(1, :), q_waypoints(2, :), '-o', ...
    'LineWidth', 1.8, 'MarkerSize', 7);
grid on;
axis equal;
xlabel('q_1');
ylabel('q_2');
title('Waypoint Connection in Joint Space');

%% Basic trajectory checks
tol = 1e-10;

for i = 1:size(q_waypoints, 2)
    t = t_waypoints(i);
    [q_chk, qd_chk, qdd_chk] = EvalMultiSegmentCubic(traj, t);

    if norm(q_chk - q_waypoints(:, i)) > tol
        error('main_test_traj:WaypointMismatch', 'Trajectory does not pass waypoint %d.', i);
    end

    if i == 1 || i == size(q_waypoints, 2)
        if norm(qd_chk) > 1e-8
            warning('main_test_traj:EndpointVelocity', 'Endpoint velocity is not near zero at waypoint %d.', i);
        end
    end
end

disp('Trajectory test completed successfully.');
