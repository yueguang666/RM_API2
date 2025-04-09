% 读取并绘制关节数据的MATLAB脚本，仅分析控制实时性
clc
clear
close all

% 文件名
filename = 'joint_data_20250408_123027.txt';

% 使用更灵活的方式读取数据
opts = detectImportOptions(filename);
opts.DataLines = [2, inf];  % 从第2行开始读取数据
data = readtable(filename, opts);

% 根据列索引获取数据
point_index = data{:, 1};
timestamps = data{:, 2};
canfd_call_timestamp_us = data{:, 21};  % 读取canfd调用时间戳

% 计算相对时间（秒）- 时间戳现在是微秒单位
relative_time = (timestamps - timestamps(1)) / 1000000;

% 分析控制实时性
canfd_intervals = zeros(length(canfd_call_timestamp_us)-1, 1);
point_index_intervals = zeros(length(point_index)-1, 1);
control_cycle = zeros(length(canfd_call_timestamp_us)-1, 1);
valid_indices = [];

for i = 2:length(canfd_call_timestamp_us)
    canfd_intervals(i-1) = (canfd_call_timestamp_us(i) - canfd_call_timestamp_us(i-1)) / 1000000;  % 转换为秒
    point_index_intervals(i-1) = point_index(i) - point_index(i-1);
    
    % 只计算有效的控制周期（避免除以零）
    if point_index_intervals(i-1) > 0
        control_cycle(i-1) = canfd_intervals(i-1) / point_index_intervals(i-1);
        valid_indices = [valid_indices; i-1];
    end
end

% 只使用有效的控制周期进行统计
valid_control_cycle = control_cycle(valid_indices);

try
    % 设置默认图形渲染器，避免使用OpenGL
    set(0,'DefaultFigureRenderer','painters');
    
    % 仅绘制每点控制周期
    figure('Name', '控制实时性分析', 'Position', [100, 100, 800, 500]);
    plot(relative_time(valid_indices+1), valid_control_cycle * 1000, 'g-', 'LineWidth', 1.5);  % 转换为毫秒显示
    title('每点控制周期分析');
    xlabel('时间 (秒)');
    ylabel('控制周期 (毫秒/点)');
    grid on;
    
    % 添加平均值线
    hold on;
    mean_cycle = mean(valid_control_cycle) * 1000;
    plot([relative_time(1), relative_time(end)], [mean_cycle, mean_cycle], 'r--', 'LineWidth', 1.2);
    legend('每点控制周期', '平均控制周期');
    
catch e
    warning('图形绘制失败: %s', e.message);
    fprintf('您可能在无图形界面的环境中运行，跳过图形绘制部分\n');
end

% 计算控制实时性的统计数据
mean_control_cycle = mean(valid_control_cycle) * 1000;  % 毫秒
std_control_cycle = std(valid_control_cycle) * 1000;    % 毫秒
min_control_cycle = min(valid_control_cycle) * 1000;    % 毫秒
max_control_cycle = max(valid_control_cycle) * 1000;    % 毫秒

fprintf('数据处理完成。共处理 %d 个数据点，时间范围 %.2f 秒\n', length(timestamps), relative_time(end));
fprintf('控制实时性分析：\n');
fprintf('  有效控制周期数据点: %d (总计 %d 个点的差值)\n', length(valid_indices), length(control_cycle));
fprintf('  平均控制周期: %.3f 毫秒/点\n', mean_control_cycle);
fprintf('  控制周期标准差: %.3f 毫秒/点\n', std_control_cycle);
fprintf('  最小控制周期: %.3f 毫秒/点\n', min_control_cycle);
fprintf('  最大控制周期: %.3f 毫秒/点\n', max_control_cycle);

% 打印点索引间隔的统计信息
fprintf('点索引间隔分析：\n');
fprintf('  最小间隔: %d\n', min(point_index_intervals));
fprintf('  最大间隔: %d\n', max(point_index_intervals));
fprintf('  零间隔数量: %d\n', sum(point_index_intervals == 0));