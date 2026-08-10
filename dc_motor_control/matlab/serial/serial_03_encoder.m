%% log_dc_motor_open_loop.m
%LOG_DC_MOTOR_OPEN_LOOP 记录 STM32 DC motor open-loop identification 数据
%
% Notes
% -----
% STM32 UART 输出格式应为：
%
%   time_ms,duty_permille,count,delta_count,rpm_x1000,rad_s_x1000
%
% 例如：
%
%   100,0,0,0,0,0
%   200,500,1234,85,13076,1369
%
% 本脚本完成：
%   1. 从串口读取 STM32 数据；
%   2. 解析 PWM duty 和速度；
%   3. 保存 CSV 和 MAT；
%   4. 绘制 open-loop input-output 曲线；
%   5. 准备一份可用于系统辨识的数据结构。

clear;
clc;
close all;

%% User configuration
% Parameters
% ----------
% port : string
%     串口号。Windows 常见为 "COM3", "COM4", "COM5"。
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
output_prefix = "dc_motor_open_loop_log";
enable_live_plot = true;

%% Open serial port
serial_obj = serialport(port, baudrate);
configureTerminator(serial_obj, "CR/LF");
serial_obj.Timeout = 1.0;
flush(serial_obj);

fprintf("Serial port opened: %s\n", port);
fprintf("Baudrate: %d\n", baudrate);
fprintf("Logging duration: %.2f s\n", duration_s);
fprintf("Expected format: time_ms,duty_p0ermille,count,delta_count,rpm_x1000,rad_s_x1000\n");

%% Data buffers
time_ms_buffer = [];
duty_permille_buffer = [];
encoder_count_buffer = [];
delta_count_buffer = [];
rpm_x1000_buffer = [];
rad_s_x1000_buffer = [];
host_time_s_buffer = [];

%% Optional live plot setup
if enable_live_plot
    fig = figure("Name", "DC motor open-loop logging");
    tiledlayout(2, 1);

    ax1 = nexttile;
    duty_line = animatedline(ax1, "LineWidth", 1.2);
    grid(ax1, "on");
    xlabel(ax1, "Time [s]");
    ylabel(ax1, "Duty [permille]");
    title(ax1, "PWM duty");

    ax2 = nexttile;
    speed_line = animatedline(ax2, "LineWidth", 1.2);
    grid(ax2, "on");
    xlabel(ax2, "Time [s]");
    ylabel(ax2, "Speed [rad/s]");
    title(ax2, "Motor speed");
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
        rpm_x1000 = values(5);
        rad_s_x1000 = values(6);

        time_ms_buffer(end + 1, 1) = time_ms;
        duty_permille_buffer(end + 1, 1) = duty_permille;
        encoder_count_buffer(end + 1, 1) = encoder_count;
        delta_count_buffer(end + 1, 1) = delta_count;
        rpm_x1000_buffer(end + 1, 1) = rpm_x1000;
        rad_s_x1000_buffer(end + 1, 1) = rad_s_x1000;
        host_time_s_buffer(end + 1, 1) = toc;

        if enable_live_plot
            t_s_live = time_ms * 1e-3;
            rad_s_live = rad_s_x1000 * 1e-3;

            addpoints(duty_line, t_s_live, duty_permille);
            addpoints(speed_line, t_s_live, rad_s_live);
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

rpm = rpm_x1000_buffer / 1000.0;
rad_per_second = rad_s_x1000_buffer / 1000.0;

sample_time_s = [NaN; diff(time_s)];

if numel(sample_time_s) >= 2
    sample_time_s(1) = median(sample_time_s(2:end), "omitnan");
else
    sample_time_s(1) = NaN;
end

%% Build table
data_table = table( ...
    time_s, ...
    duty_permille_buffer, ...
    duty_ratio, ...
    encoder_count_buffer, ...
    delta_count_buffer, ...
    rpm, ...
    rad_per_second, ...
    sample_time_s, ...
    host_time_s_buffer, ...
    'VariableNames', { ...
        'time_s', ...
        'duty_permille', ...
        'duty_ratio', ...
        'encoder_count', ...
        'delta_count', ...
        'rpm', ...
        'rad_per_second', ...
        'sample_time_s', ...
        'host_time_s' ...
    } ...
);

%% Save data
csv_filename = output_prefix + ".csv";
mat_filename = output_prefix + ".mat";

writetable(data_table, csv_filename);

ident_data = struct();
ident_data.time_s = data_table.time_s;
ident_data.u_duty_permille = data_table.duty_permille;
ident_data.u_duty_ratio = data_table.duty_ratio;
ident_data.y_rpm = data_table.rpm;
ident_data.y_rad_per_second = data_table.rad_per_second;
ident_data.encoder_count = data_table.encoder_count;
ident_data.delta_count = data_table.delta_count;
ident_data.sample_time_s = data_table.sample_time_s;

save(mat_filename, "data_table", "ident_data");

fprintf("Saved CSV: %s\n", csv_filename);
fprintf("Saved MAT: %s\n", mat_filename);

%% Plot final results
figure("Name", "PWM duty input");
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("PWM duty [permille]");
title("PWM duty input");

figure("Name", "Motor speed in rad/s");
plot(data_table.time_s, data_table.rad_per_second, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Speed [rad/s]");
title("Motor speed");

figure("Name", "Motor speed in rpm");
plot(data_table.time_s, data_table.rpm, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Speed [rpm]");
title("Motor speed");

figure("Name", "Open-loop input-output response");
yyaxis left;
plot(data_table.time_s, data_table.duty_permille, "LineWidth", 1.2);
ylabel("PWM duty [permille]");

yyaxis right;
plot(data_table.time_s, data_table.rad_per_second, "LineWidth", 1.2);
ylabel("Speed [rad/s]");

grid on;
xlabel("Time [s]");
title("Open-loop PWM input and speed response");

figure("Name", "Sample time check");
plot(data_table.time_s, data_table.sample_time_s * 1000.0, "LineWidth", 1.2);
grid on;
xlabel("Time [s]");
ylabel("Sample time [ms]");
title("UART logged sample time");

%% Quick identification data preview
% Notes
% -----
% 后续做辨识时，可以使用：
%
%   u = ident_data.u_duty_ratio;
%   y = ident_data.y_rad_per_second;
%   t = ident_data.time_s;
%
% 如果使用 System Identification Toolbox，可以进一步构造 iddata：
%
%   Ts = median(data_table.sample_time_s, "omitnan");
%   z = iddata(y, u, Ts);
%
% 但当前脚本不强依赖 System Identification Toolbox。
u = ident_data.u_duty_ratio;
y = ident_data.y_rad_per_second;
t = ident_data.time_s;

fprintf("Data preview:\n");
fprintf("  time range: %.3f ~ %.3f s\n", t(1), t(end));
fprintf("  duty range: %.3f ~ %.3f\n", min(u), max(u));
fprintf("  speed range: %.3f ~ %.3f rad/s\n", min(y), max(y));
fprintf("  median sample time: %.3f ms\n", median(data_table.sample_time_s, "omitnan") * 1000.0);


%%
%% create iddata object
Ts_id = median(data_table.sample_time_s, "omitnan");

u_id = data_table.duty_ratio - data_table.duty_ratio(floor(10/Ts_id));
y_id = data_table.rad_per_second - data_table.rad_per_second(floor(10/Ts_id));

z_speed = iddata(y_id, u_id, Ts_id);
z_speed.InputName = "PWM duty ratio";
z_speed.OutputName = "Motor speed";
z_speed.InputUnit = "1";
z_speed.OutputUnit = "rad/s";
z_speed.TimeUnit = "s";

save("chirp_iddata.mat", "z_speed");

figure
plot(z_speed);

%%
save("chirp_model.mat", "P1D_chirp")