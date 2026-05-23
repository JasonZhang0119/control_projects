%% log_dc_motor_position_speed_control.m
%LOG_DC_MOTOR_POSITION_SPEED_CONTROL 记录 STM32 DC motor position/speed control 数据
%
% Notes
% -----
% STM32 UART 输出格式应为：
%
%   time_ms,duty_permille,count,delta_count,speed_ref_x1000,speed_meas_x1000,position_ref_x1000,position_meas_x1000
%
% 对应 STM32 端：
%
%   std::snprintf(
%       tx_buf,
%       sizeof(tx_buf),
%       "%lu,%ld,%u,%d,%ld,%ld,%ld,%ld\r\n",
%       static_cast<unsigned long>(now_tick_ms),
%       static_cast<long>(signed_duty_permille),
%       static_cast<unsigned int>(encoder_meas.count),
%       static_cast<int>(encoder_meas.delta_count),
%       static_cast<long>(this->speed_ref_rad_s_ * 1000.0F),
%       static_cast<long>(encoder_meas.rad_per_second * 1000.0F),
%       static_cast<long>(this->position_ref_rad_ * 1000.0F),
%       static_cast<long>(encoder_meas.position_rad * 1000.0F)
%   );
%
% 本脚本完成：
%   1. 从串口读取 STM32 数据；
%   2. 解析 PWM duty、encoder count、delta count；
%   3. 解析 speed_ref、speed_meas、position_ref、position_meas；
%   4. 保存 CSV 和 MAT；
%   5. 绘制速度跟踪、位置跟踪、duty 和采样时间。

clear;
clc;
close all;

%% User configuration
% Parameters
% ----------
% port : string
%     串口号。Windows 常见为 "COM3", "COM4", "COM8"。
% baudrate : double
%     STM32 UART 波特率。
% duration_s : double
%     数据采集时长，单位 s。
% output_prefix : string
%     输出文件名前缀。
% enable_live_plot : logical
%     是否启用实时绘图。
port = "COM8";
baudrate = 115200;
duration_s = 30.0;
output_prefix = "dc_motor_position_speed_control_log";
enable_live_plot = true;

%% Open serial port
serial_obj = serialport(port, baudrate);
configureTerminator(serial_obj, "CR/LF");
serial_obj.Timeout = 1.0;
flush(serial_obj);

fprintf("Serial port opened: %s\n", port);
fprintf("Baudrate: %d\n", baudrate);
fprintf("Logging duration: %.2f s\n", duration_s);
fprintf("Expected format: time_ms,duty_permille,count,delta_count,speed_ref,speed_meas,position_ref,position_meas\n");

%% Data buffers
time_ms_buffer = [];
duty_permille_buffer = [];
encoder_count_buffer = [];
delta_count_buffer = [];
speed_ref_buffer = [];
speed_meas_buffer = [];
position_ref_buffer = [];
position_meas_buffer = [];
host_time_s_buffer = [];

%% Optional live plot setup
if enable_live_plot
    fig = figure("Name", "DC motor position/speed control logging");
    tiledlayout(4, 1);

    ax1 = nexttile;
    duty_line = animatedline(ax1, "LineWidth", 1.2);
    grid(ax1, "on");
    xlabel(ax1, "Time [s]");
    ylabel(ax1, "Duty [permille]");
    title(ax1, "PWM duty");

    ax2 = nexttile;
    speed_ref_line = animatedline(ax2, "LineWidth", 1.2);
    speed_meas_line = animatedline(ax2, "LineWidth", 1.2);
    grid(ax2, "on");
    xlabel(ax2, "Time [s]");
    ylabel(ax2, "Speed [rad/s]");
    title(ax2, "Speed reference and measurement");
    legend(ax2, "speed ref", "speed meas");

    ax3 = nexttile;
    position_ref_line = animatedline(ax3, "LineWidth", 1.2);
    position_meas_line = animatedline(ax3, "LineWidth", 1.2);
    grid(ax3, "on");
    xlabel(ax3, "Time [s]");
    ylabel(ax3, "Position [rad]");
    title(ax3, "Position reference and measurement");
    legend(ax3, "position ref", "position meas");

    ax4 = nexttile;
    delta_line = animatedline(ax4, "LineWidth", 1.2);
    grid(ax4, "on");
    xlabel(ax4, "Time [s]");
    ylabel(ax4, "Delta count");
    title(ax4, "Encoder delta count");
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

        values = sscanf(raw_line, "%f,%f,%f,%f,%f,%f,%f,%f");

        if numel(values) ~= 8
            num_invalid_lines = num_invalid_lines + 1;
            fprintf("Skip invalid line: %s\n", raw_line);
            continue;
        end

        time_ms = values(1);
        duty_permille = values(2);
        encoder_count = values(3);
        delta_count = values(4);
        speed_ref_x1000 = values(5);
        speed_meas_x1000 = values(6);
        position_ref_x1000 = values(7);
        position_meas_x1000 = values(8);

        speed_ref = speed_ref_x1000 / 1000.0;
        speed_meas = speed_meas_x1000 / 1000.0;
        position_ref = position_ref_x1000 / 1000.0;
        position_meas = position_meas_x1000 / 1000.0;

        time_ms_buffer(end + 1, 1) = time_ms;
        duty_permille_buffer(end + 1, 1) = duty_permille;
        encoder_count_buffer(end + 1, 1) = encoder_count;
        delta_count_buffer(end + 1, 1) = delta_count;
        speed_ref_buffer(end + 1, 1) = speed_ref;
        speed_meas_buffer(end + 1, 1) = speed_meas;
        position_ref_buffer(end + 1, 1) = position_ref;
        position_meas_buffer(end + 1, 1) = position_meas;
        host_time_s_buffer(end + 1, 1) = toc;

        if enable_live_plot
            t_s_live = time_ms * 1e-3;

            addpoints(duty_line, t_s_live, duty_permille);
            addpoints(speed_ref_line, t_s_live, speed_ref);
            addpoints(speed_meas_line, t_s_live, speed_meas);
            addpoints(position_ref_line, t_s_live, position_ref);
            addpoints(position_meas_line, t_s_live, position_meas);
            addpoints(delta_line, t_s_live, delta_count);

            drawnow limitrate;
        end

    catch ME
        fprintf("Read warning: %s\n", ME.message);
    end
end

clear serial_obj;

fprintf("Logging finished.\n");
fprintf("Valid samples: %d\n", numel(time_ms_buffer));
fprintf("Invalid lines: %d\n", num_invalid_lines);

%% Validate data
if numel(time_ms_buffer) < 5
    error("有效数据太少。请检查串口号、波特率、STM32 输出格式和接线。");
end

%% Post-processing
time_s = time_ms_buffer * 1e-3;
time_s = time_s - time_s(1);

duty_ratio = duty_permille_buffer / 1000.0;

sample_time_s = [NaN; diff(time_s)];

if numel(sample_time_s) >= 2
    sample_time_s(1) = median(sample_time_s(2:end), "omitnan");
else
    sample_time_s(1) = NaN;
end

speed_error = speed_ref_buffer - speed_meas_buffer;
position_error = position_ref_buffer - position_meas_buffer;

%% Build table
data_table = table( ...
    time_s, ...
    duty_permille_buffer, ...
    duty_ratio, ...
    encoder_count_buffer, ...
    delta_count_buffer, ...
    speed_ref_buffer, ...
    speed_meas_buffer, ...
    speed_error, ...
    position_ref_buffer, ...
    position_meas_buffer, ...
    position_error, ...
    sample_time_s, ...
    host_time_s_buffer, ...
    'VariableNames', { ...
        'time_s', ...
        'duty_permille', ...
        'duty_ratio', ...
        'encoder_count', ...
        'delta_count', ...
        'speed_ref_rad_s', ...
        'speed_meas_rad_s', ...
        'speed_error_rad_s', ...
        'position_ref_rad', ...
        'position_meas_rad', ...
        'position_error_rad', ...
        'sample_time_s', ...
        'host_time_s' ...
    } ...
);

%% Save data
csv_filename = output_prefix + ".csv";
mat_filename = output_prefix + ".mat";

writetable(data_table, csv_filename);

control_data = struct();
control_data.time_s = data_table.time_s;
control_data.u_duty_permille = data_table.duty_permille;
control_data.u_duty_ratio = data_table.duty_ratio;
control_data.encoder_count = data_table.encoder_count;
control_data.delta_count = data_table.delta_count;
control_data.speed_ref_rad_s = data_table.speed_ref_rad_s;
control_data.speed_meas_rad_s = data_table.speed_meas_rad_s;
control_data.speed_error_rad_s = data_table.speed_error_rad_s;
control_data.position_ref_rad = data_table.position_ref_rad;
control_data.position_meas_rad = data_table.position_meas_rad;
control_data.position_error_rad = data_table.position_error_rad;
control_data.sample_time_s = data_table.sample_time_s;
control_data.host_time_s = data_table.host_time_s;

save(mat_filename, "data_table", "control_data");

fprintf("Saved CSV: %s\n", csv_filename);
fprintf("Saved MAT: %s\n", mat_filename);

%% Plot final results
figure("Name", "PWM duty input");
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("PWM duty [permille]");
title("PWM duty input");

figure("Name", "Speed tracking");
plot(data_table.time_s, data_table.speed_ref_rad_s, 'r', "LineWidth", 2);
hold on;
plot(data_table.time_s, data_table.speed_meas_rad_s, 'b--', "LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Speed [rad/s]");
title("Speed reference and measured speed");
legend("speed ref", "speed meas");

figure("Name", "Speed tracking error");
plot(data_table.time_s, data_table.speed_error_rad_s, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Speed error [rad/s]");
title("Speed tracking error");

figure("Name", "Position tracking");
plot(data_table.time_s, data_table.position_ref_rad, 'r', "LineWidth", 2);
hold on;
plot(data_table.time_s, data_table.position_meas_rad, 'b--', "LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Position [rad]");
title("Position reference and measured position");
legend("position ref", "position meas");

figure("Name", "Position tracking error");
plot(data_table.time_s, data_table.position_error_rad, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Position error [rad]");
title("Position tracking error");

figure("Name", "Duty and speed response");
yyaxis left;
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
ylabel("PWM duty [permille]");

yyaxis right;
plot(data_table.time_s, data_table.speed_ref_rad_s, "--", "LineWidth", 1.2);
hold on;
plot(data_table.time_s, data_table.speed_meas_rad_s, "LineWidth", 1.2);
ylabel("Speed [rad/s]");

grid on;
xlabel("Time [s]");
title("PWM duty and speed response");
legend("duty", "speed ref", "speed meas");

figure("Name", "Duty and position response");
yyaxis left;
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
ylabel("PWM duty [permille]");

yyaxis right;
plot(data_table.time_s, data_table.position_ref_rad, "--", "LineWidth", 1.2);
hold on;
plot(data_table.time_s, data_table.position_meas_rad, "LineWidth", 1.2);
ylabel("Position [rad]");

grid on;
xlabel("Time [s]");
title("PWM duty and position response");
legend("duty", "position ref", "position meas");

figure("Name", "Encoder delta count");
plot(data_table.time_s, data_table.delta_count, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Delta count");
title("Encoder delta count");

figure("Name", "Encoder count");
plot(data_table.time_s, data_table.encoder_count, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Encoder count");
title("Encoder count");

figure("Name", "Sample time check");
plot(data_table.time_s, data_table.sample_time_s * 1000.0, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Sample time [ms]");
title("UART logged sample time");

%% Data preview
fprintf("Data preview:\n");
fprintf("  time range: %.3f ~ %.3f s\n", data_table.time_s(1), data_table.time_s(end));
fprintf("  duty range: %.3f ~ %.3f permille\n", min(data_table.duty_permille), max(data_table.duty_permille));
fprintf("  speed ref range: %.3f ~ %.3f rad/s\n", min(data_table.speed_ref_rad_s), max(data_table.speed_ref_rad_s));
fprintf("  speed meas range: %.3f ~ %.3f rad/s\n", min(data_table.speed_meas_rad_s), max(data_table.speed_meas_rad_s));
fprintf("  speed error range: %.3f ~ %.3f rad/s\n", min(data_table.speed_error_rad_s), max(data_table.speed_error_rad_s));
fprintf("  position ref range: %.3f ~ %.3f rad\n", min(data_table.position_ref_rad), max(data_table.position_ref_rad));
fprintf("  position meas range: %.3f ~ %.3f rad\n", min(data_table.position_meas_rad), max(data_table.position_meas_rad));
fprintf("  position error range: %.3f ~ %.3f rad\n", min(data_table.position_error_rad), max(data_table.position_error_rad));
fprintf("  median sample time: %.3f ms\n", median(data_table.sample_time_s, "omitnan") * 1000.0);