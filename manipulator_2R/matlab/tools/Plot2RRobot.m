function h = Plot2RRobot(q, param, options, h)
%PLOT2RROBOT 绘制或更新平面 2R 机械臂
%
% 输入
% ----
% q : double, size (2, 1)
%     当前关节角向量 [q1; q2]，单位 rad
%
% param : struct
%     机械臂参数结构体，至少需要包含：
%         l1 : 第一根连杆长度
%         l2 : 第二根连杆长度
%
% options : struct, 可选
%     绘图选项：
%         ee_traj      : 实际末端轨迹，size (2, N)
%         ee_ref_traj  : 参考末端轨迹，size (2, N)
%         axis_limit   : 坐标轴范围 [xmin xmax ymin ymax]
%         title_text   : 图标题
%         show_frame   : 是否显示坐标轴方向
%
% h : struct, 可选
%     已有图形句柄结构体。若传入，则更新已有图像。
%
% 输出
% ----
% h : struct
%     图形句柄结构体，用于后续动画刷新

    if nargin < 3 || isempty(options)
        options = struct();
    end

    if nargin < 4
        h = struct();
    end

    if ~isfield(options, 'ee_traj')
        options.ee_traj = [];
    end

    if ~isfield(options, 'ee_ref_traj')
        options.ee_ref_traj = [];
    end

    if ~isfield(options, 'axis_limit')
        options.axis_limit = [];
    end

    if ~isfield(options, 'title_text')
        options.title_text = 'Planar 2R Robot';
    end

    if ~isfield(options, 'show_frame')
        options.show_frame = true;
    end

    q = q(:);

    if numel(q) ~= 2
        error('Plot2RRobot:DimensionMismatch', ...
              'q 必须是长度为 2 的关节角向量。');
    end

    l1 = param.l1;
    l2 = param.l2;

    q1 = q(1);
    q2 = q(2);

    % 机器人基座、肘部和末端位置
    p0 = [0; 0];
    p1 = [l1 * cos(q1);
          l1 * sin(q1)];
    pe = [l1 * cos(q1) + l2 * cos(q1 + q2);
          l1 * sin(q1) + l2 * sin(q1 + q2)];

    robot_x = [p0(1), p1(1), pe(1)];
    robot_y = [p0(2), p1(2), pe(2)];

    if isempty(options.axis_limit)
        total_length = l1 + l2;
        options.axis_limit = 1.2 * [-total_length, total_length, ...
                                    -total_length, total_length];
    end

    is_new_plot = ~isfield(h, 'robot_line') || ~isvalid(h.robot_line);

    if is_new_plot
        hold on;
        grid on;
        axis equal;
        axis(options.axis_limit);

        % 机械臂连线
        h.robot_line = plot(robot_x, robot_y, '-o', ...
            'LineWidth', 2, ...
            'MarkerSize', 6);

        % 末端点
        h.ee_point = plot(pe(1), pe(2), 'o', ...
            'MarkerSize', 8, ...
            'LineWidth', 2);

        % 实际末端轨迹
        h.ee_traj = plot(nan, nan, '--', ...
            'LineWidth', 1.5);

        % 参考末端轨迹
        h.ee_ref_traj = plot(nan, nan, ':', ...
            'LineWidth', 1.5);

        h.title = title(options.title_text);
        xlabel('x');
        ylabel('y');

        if options.show_frame
            frame_scale = 0.2;

            % 基座坐标系方向
            h.frame0 = quiver(0, 0, frame_scale, 0, 0, ...
                'LineWidth', 1.2);

            % 第一连杆坐标系方向
            h.frame1 = quiver(p1(1), p1(2), ...
                frame_scale * cos(q1), ...
                frame_scale * sin(q1), ...
                0, 'LineWidth', 1.2);

            % 末端坐标系方向
            h.framee = quiver(pe(1), pe(2), ...
                frame_scale * cos(q1 + q2), ...
                frame_scale * sin(q1 + q2), ...
                0, 'LineWidth', 1.2);
        end

        legend({'robot', 'end-effector', 'actual traj', 'reference traj'}, ...
               'Location', 'bestoutside');

    else
        set(h.robot_line, 'XData', robot_x, 'YData', robot_y);
        set(h.ee_point, 'XData', pe(1), 'YData', pe(2));
    end

    if ~isempty(options.ee_traj)
        set(h.ee_traj, ...
            'XData', options.ee_traj(1, :), ...
            'YData', options.ee_traj(2, :));
    end

    if ~isempty(options.ee_ref_traj)
        set(h.ee_ref_traj, ...
            'XData', options.ee_ref_traj(1, :), ...
            'YData', options.ee_ref_traj(2, :));
    end

    if options.show_frame && isfield(h, 'frame1')
        frame_scale = 0.2;

        set(h.frame1, ...
            'XData', p1(1), ...
            'YData', p1(2), ...
            'UData', frame_scale * cos(q1), ...
            'VData', frame_scale * sin(q1));

        set(h.framee, ...
            'XData', pe(1), ...
            'YData', pe(2), ...
            'UData', frame_scale * cos(q1 + q2), ...
            'VData', frame_scale * sin(q1 + q2));
    end

    drawnow;
end
