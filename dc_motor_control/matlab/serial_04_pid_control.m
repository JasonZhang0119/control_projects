%% log_dc_motor_speed_control.m
%LOG_DC_MOTOR_SPEED_CONTROL 记录 STM32 DC motor speed control 数据
%
% Notes
% -----
% STM32 UART 输出格式应为：
%
%   time_ms,duty_permille,count,delta_count,speed_ref,speed_meas
%
% 对应 STM32 端：
%
%   std::snprintf(
%       tx_buf,
%       sizeof(tx_buf),
%       "%lu,%lu,%u,%d,%ld,%ld\r\n",
%       static_cast<unsigned long>(now_tick),
%       static_cast<unsigned long>(duty_permille),
%       static_cast<unsigned int>(encoder_meas.count),
%       static_cast<int>(encoder_meas.delta_count),
%       static_cast<long>(speed_ref(0)),
%       static_cast<long>(speed_meas(0))
%   );
%
% 本脚本完成：
%   1. 从串口读取 STM32 数据；
%   2. 解析 PWM duty、encoder count、delta count、speed_ref、speed_meas；
%   3. 保存 CSV 和 MAT；
%   4. 绘制 reference、measured speed 和 duty；
%   5. 准备后续闭环控制分析数据。

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
duration_s = 100.0;
output_prefix = "dc_motor_speed_control_log";
enable_live_plot = true;

%% Open serial port
serial_obj = serialport(port, baudrate);
configureTerminator(serial_obj, "CR/LF");
serial_obj.Timeout = 1.0;
flush(serial_obj);

fprintf("Serial port opened: %s\n", port);
fprintf("Baudrate: %d\n", baudrate);
fprintf("Logging duration: %.2f s\n", duration_s);
fprintf("Expected format: time_ms,duty_permille,count,delta_count,speed_ref,speed_meas\n");

%% Data buffers
time_ms_buffer = [];
duty_permille_buffer = [];
encoder_count_buffer = [];
delta_count_buffer = [];
speed_ref_buffer = [];
speed_meas_buffer = [];
host_time_s_buffer = [];

%% Optional live plot setup
if enable_live_plot
    fig = figure("Name", "DC motor speed control logging");
    tiledlayout(3, 1);

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
    ylabel(ax2, "Speed");
    title(ax2, "Speed reference and measurement");
    legend(ax2, "speed ref", "speed meas");

    ax3 = nexttile;
    delta_line = animatedline(ax3, "LineWidth", 1.2);
    grid(ax3, "on");
    xlabel(ax3, "Time [s]");
    ylabel(ax3, "Delta count");
    title(ax3, "Encoder delta count");
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

        values = sscanf(raw_line, "%f,%f,%f,%f,%f,%f");

        if numel(values) ~= 6
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

        speed_ref = speed_ref_x1000 / 1000.0;
        speed_meas = speed_meas_x1000 / 1000.0;

        time_ms_buffer(end + 1, 1) = time_ms;
        duty_permille_buffer(end + 1, 1) = duty_permille;
        encoder_count_buffer(end + 1, 1) = encoder_count;
        delta_count_buffer(end + 1, 1) = delta_count;
        speed_ref_buffer(end + 1, 1) = speed_ref;
        speed_meas_buffer(end + 1, 1) = speed_meas;
        host_time_s_buffer(end + 1, 1) = toc;

        if enable_live_plot
            t_s_live = time_ms * 1e-3;

            addpoints(duty_line, t_s_live, duty_permille);
            addpoints(speed_ref_line, t_s_live, speed_ref);
            addpoints(speed_meas_line, t_s_live, speed_meas);
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
    sample_time_s, ...
    host_time_s_buffer, ...
    'VariableNames', { ...
        'time_s', ...
        'duty_permille', ...
        'duty_ratio', ...
        'encoder_count', ...
        'delta_count', ...
        'speed_ref', ...
        'speed_meas', ...
        'speed_error', ...
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
control_data.speed_ref = data_table.speed_ref;
control_data.speed_meas = data_table.speed_meas;
control_data.speed_error = data_table.speed_error;
control_data.sample_time_s = data_table.sample_time_s;

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
plot(data_table.time_s, data_table.speed_ref, 'r', "LineWidth", 2);
hold on;
plot(data_table.time_s, data_table.speed_meas, 'b--', "LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Speed");
title("Speed reference and measured speed");
legend("speed ref", "speed meas");

figure("Name", "Speed tracking error");
plot(data_table.time_s, data_table.speed_error, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Speed error");
title("Speed tracking error");

figure("Name", "Duty and speed response");
yyaxis left;
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
ylabel("PWM duty [permille]");

yyaxis right;
plot(data_table.time_s, data_table.speed_ref, "--", "LineWidth", 1.2);
hold on;
plot(data_table.time_s, data_table.speed_meas, "LineWidth", 1.2);
ylabel("Speed");

grid on;
xlabel("Time [s]");
title("PWM duty and speed response");
legend("duty", "speed ref", "speed meas");

figure("Name", "Encoder delta count");
plot(data_table.time_s, data_table.delta_count, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Delta count");
title("Encoder delta count");

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
fprintf("  speed ref range: %.3f ~ %.3f\n", min(data_table.speed_ref), max(data_table.speed_ref));
fprintf("  speed meas range: %.3f ~ %.3f\n", min(data_table.speed_meas), max(data_table.speed_meas));
fprintf("  speed error range: %.3f ~ %.3f\n", min(data_table.speed_error), max(data_table.speed_error));
fprintf("  median sample time: %.3f ms\n", median(data_table.sample_time_s, "omitnan") * 1000.0);