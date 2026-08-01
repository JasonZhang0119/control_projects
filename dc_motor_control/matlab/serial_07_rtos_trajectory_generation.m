%% log_rtos_dc_motor_control.m
%LOG_RTOS_DC_MOTOR_CONTROL 记录 STM32 FreeRTOS DC motor control 数据
%
% STM32 UART 输出格式：
%
%   dt_ms,speed_ref_x1000,speed_meas_x1000,position_ref_x1000,position_meas_x1000,position_ref_filtered_x1000,duty_cycle_x1000

clear;
clc;
close all;

%% User configuration
port = "COM8";
baudrate = 115200;
duration_s = 60.0;
output_prefix = "rtos_dc_motor_control_log";
enable_live_plot = true;

%% Open serial port
serial_obj = serialport(port, baudrate);
configureTerminator(serial_obj, "CR/LF");
serial_obj.Timeout = 1.0;
flush(serial_obj);

fprintf("Serial port opened: %s\n", port);
fprintf("Baudrate: %d\n", baudrate);
fprintf("Logging duration: %.2f s\n", duration_s);
fprintf("Expected format: dt_ms,speed_ref,speed_meas,position_ref,position_meas,position_ref_filtered,duty_cycle\n");

%% Data buffers
dt_ms_buffer = [];
speed_ref_buffer = [];
speed_meas_buffer = [];
position_ref_buffer = [];
position_meas_buffer = [];
position_ref_filtered_buffer = [];
duty_cycle_buffer = [];
host_time_s_buffer = [];

%% Live plot
if enable_live_plot
    fig = figure("Name", "RTOS DC motor control logging");
    tiledlayout(4, 1);

    ax1 = nexttile;
    speed_ref_line = animatedline(ax1, "LineWidth", 1.2, "Color", "r");
    speed_meas_line = animatedline(ax1, "LineWidth", 1.2, "Color", "b", "LineStyle", "--");
    grid(ax1, "on");
    xlabel(ax1, "Time [s]");
    ylabel(ax1, "Speed [rad/s]");
    title(ax1, "Speed reference and measurement");
    legend(ax1, "speed ref", "speed meas");

    ax2 = nexttile;
    position_ref_line = animatedline(ax2, "LineWidth", 1.2, "Color", "k");
    position_ref_filtered_line = animatedline(ax2, "LineWidth", 1.2, "Color", "r");
    position_meas_line = animatedline(ax2, "LineWidth", 1.2, "Color", "b", "LineStyle", "--");
    grid(ax2, "on");
    xlabel(ax2, "Time [s]");
    ylabel(ax2, "Position [rad]");
    title(ax2, "Position reference, filtered reference and measurement");
    legend(ax2, "position ref", "filtered ref", "position meas");

    ax3 = nexttile;
    duty_cycle_line = animatedline(ax3, "LineWidth", 1.2, "Color", "b");
    grid(ax3, "on");
    xlabel(ax3, "Time [s]");
    ylabel(ax3, "Duty cycle");
    title(ax3, "PWM duty cycle");

    ax4 = nexttile;
    dt_line = animatedline(ax4, "LineWidth", 1.2, "Color", "b");
    grid(ax4, "on");
    xlabel(ax4, "Time [s]");
    ylabel(ax4, "dt [ms]");
    title(ax4, "Logger sample interval");
end

%% Logging loop
tic;
num_invalid_lines = 0;

while toc < duration_s
    try
        raw_line = readline(serial_obj);
        raw_line = strtrim(raw_line);

        if strlength(raw_line) == 0
            continue;
        end

        values = sscanf(raw_line, "%f,%f,%f,%f,%f,%f,%f");

        if numel(values) ~= 7
            num_invalid_lines = num_invalid_lines + 1;
            fprintf("Skip invalid line: %s\n", raw_line);
            continue;
        end

        dt_ms = values(1);
        speed_ref = values(2) / 1000.0;
        speed_meas = values(3) / 1000.0;
        position_ref = values(4) / 1000.0;
        position_meas = values(5) / 1000.0;
        position_ref_filtered = values(6) / 1000.0;
        duty_cycle = values(7) / 1000.0;

        dt_ms_buffer(end + 1, 1) = dt_ms;
        speed_ref_buffer(end + 1, 1) = speed_ref;
        speed_meas_buffer(end + 1, 1) = speed_meas;
        position_ref_buffer(end + 1, 1) = position_ref;
        position_meas_buffer(end + 1, 1) = position_meas;
        position_ref_filtered_buffer(end + 1, 1) = position_ref_filtered;
        duty_cycle_buffer(end + 1, 1) = duty_cycle;
        host_time_s_buffer(end + 1, 1) = toc;

        if enable_live_plot
            t_s_live = host_time_s_buffer(end);

            addpoints(speed_ref_line, t_s_live, speed_ref);
            addpoints(speed_meas_line, t_s_live, speed_meas);
            addpoints(position_ref_line, t_s_live, position_ref);
            addpoints(position_ref_filtered_line, t_s_live, position_ref_filtered);
            addpoints(position_meas_line, t_s_live, position_meas);
            addpoints(duty_cycle_line, t_s_live, duty_cycle);
            addpoints(dt_line, t_s_live, dt_ms);

            drawnow limitrate;
        end

    catch ME
        fprintf("Read warning: %s\n", ME.message);
    end
end

clear serial_obj;

fprintf("Logging finished.\n");
fprintf("Valid samples: %d\n", numel(dt_ms_buffer));
fprintf("Invalid lines: %d\n", num_invalid_lines);

%% Validate data
if numel(dt_ms_buffer) < 5
    error("有效数据太少。请检查串口号、波特率、STM32输出格式和接线。");
end

%% Post-processing
time_s = host_time_s_buffer - host_time_s_buffer(1);
sample_time_s = dt_ms_buffer / 1000.0;

speed_error = speed_ref_buffer - speed_meas_buffer;
position_error = position_ref_buffer - position_meas_buffer;
position_filtered_error = position_ref_filtered_buffer - position_meas_buffer;

%% Build table
data_table = table( ...
    time_s, ...
    dt_ms_buffer, ...
    sample_time_s, ...
    speed_ref_buffer, ...
    speed_meas_buffer, ...
    speed_error, ...
    position_ref_buffer, ...
    position_ref_filtered_buffer, ...
    position_meas_buffer, ...
    position_error, ...
    position_filtered_error, ...
    duty_cycle_buffer, ...
    host_time_s_buffer, ...
    'VariableNames', { ...
        'time_s', ...
        'dt_ms', ...
        'sample_time_s', ...
        'speed_ref_rad_s', ...
        'speed_meas_rad_s', ...
        'speed_error_rad_s', ...
        'position_ref_rad', ...
        'position_ref_filtered_rad', ...
        'position_meas_rad', ...
        'position_error_rad', ...
        'position_filtered_error_rad', ...
        'duty_cycle', ...
        'host_time_s' ...
    } ...
);

%% Save data
csv_filename = output_prefix + ".csv";
mat_filename = output_prefix + ".mat";

writetable(data_table, csv_filename);

control_data = struct();
control_data.time_s = data_table.time_s;
control_data.dt_ms = data_table.dt_ms;
control_data.sample_time_s = data_table.sample_time_s;
control_data.speed_ref_rad_s = data_table.speed_ref_rad_s;
control_data.speed_meas_rad_s = data_table.speed_meas_rad_s;
control_data.speed_error_rad_s = data_table.speed_error_rad_s;
control_data.position_ref_rad = data_table.position_ref_rad;
control_data.position_ref_filtered_rad = data_table.position_ref_filtered_rad;
control_data.position_meas_rad = data_table.position_meas_rad;
control_data.position_error_rad = data_table.position_error_rad;
control_data.position_filtered_error_rad = data_table.position_filtered_error_rad;
control_data.duty_cycle = data_table.duty_cycle;
control_data.host_time_s = data_table.host_time_s;

save(mat_filename, "data_table", "control_data");

fprintf("Saved CSV: %s\n", csv_filename);
fprintf("Saved MAT: %s\n", mat_filename);

%% Final plots
figure("Name", "Speed tracking");
plot(data_table.time_s, data_table.speed_ref_rad_s, "r", "LineWidth", 2);
hold on;
plot(data_table.time_s, data_table.speed_meas_rad_s, "b--", "LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Speed [rad/s]");
title("Speed reference and measured speed");
legend("speed ref", "speed meas");

figure("Name", "Position tracking");
plot(data_table.time_s, data_table.position_ref_rad, "k", "LineWidth", 2);
hold on;
plot(data_table.time_s, data_table.position_ref_filtered_rad, "r", "LineWidth", 2);
plot(data_table.time_s, data_table.position_meas_rad, "b--", "LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Position [rad]");
title("Position reference, filtered reference and measured position");
legend("position ref", "filtered ref", "position meas");

figure("Name", "Position tracking error");
plot(data_table.time_s, data_table.position_error_rad, "b", "LineWidth", 1.2);
hold on;
plot(data_table.time_s, data_table.position_filtered_error_rad, "r", "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Position error [rad]");
title("Position tracking error");
legend("raw ref error", "filtered ref error");

figure("Name", "PWM duty cycle");
plot(data_table.time_s, data_table.duty_cycle, "b", "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Duty cycle");
title("PWM duty cycle");

figure("Name", "Sample time check");
plot(data_table.time_s, data_table.dt_ms, "b", "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("dt [ms]");
title("Logged task interval");

%% Preview
fprintf("Data preview:\n");
fprintf("  time range: %.3f ~ %.3f s\n", data_table.time_s(1), data_table.time_s(end));
fprintf("  speed ref range: %.3f ~ %.3f rad/s\n", min(data_table.speed_ref_rad_s), max(data_table.speed_ref_rad_s));
fprintf("  speed meas range: %.3f ~ %.3f rad/s\n", min(data_table.speed_meas_rad_s), max(data_table.speed_meas_rad_s));
fprintf("  position ref range: %.3f ~ %.3f rad\n", min(data_table.position_ref_rad), max(data_table.position_ref_rad));
fprintf("  filtered ref range: %.3f ~ %.3f rad\n", min(data_table.position_ref_filtered_rad), max(data_table.position_ref_filtered_rad));
fprintf("  position meas range: %.3f ~ %.3f rad\n", min(data_table.position_meas_rad), max(data_table.position_meas_rad));
fprintf("  duty cycle range: %.3f ~ %.3f\n", min(data_table.duty_cycle), max(data_table.duty_cycle));
fprintf("  median dt: %.3f ms\n", median(data_table.dt_ms, "omitnan"));