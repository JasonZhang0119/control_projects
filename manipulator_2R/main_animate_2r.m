clear; clc; close all;

param = Param2R('standard');

q0 = [0.0; 0.0];
qf = [pi/2; -pi/3];

q0_dot = [0.0; 0.0];
qf_dot = [0.0; 0.0];

q0_ddot = [0.0; 0.0];
qf_ddot = [0.0; 0.0];

t0 = 0.0;
tf = 3.0;

N = 150;
t_vec = linspace(t0, tf, N);

q_traj = zeros(2, N);

for k = 1:N
    q_traj(:, k) = QuinticInterp( ...
        q0, qf, ...
        q0_dot, qf_dot, ...
        q0_ddot, qf_ddot, ...
        t0, tf, t_vec(k));
end

options = struct();
options.dt = 0.02;
options.show_trace = true;
options.title = '2R Quintic Joint Trajectory';

Animate2R(q_traj, param, options);