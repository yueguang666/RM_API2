% 读取并绘制关节数据的MATLAB脚本
clc
clear
close all

% 文件名
filename = 'joint_data_20250403_211240_jmk控制周期2关节速度180.txt';

% 使用更灵活的方式读取数据
opts = detectImportOptions(filename);
opts.DataLines = [2, inf];  % 从第2行开始读取数据
data = readtable(filename, opts);

% 根据列索引获取数据
point_index = data{:, 1};
timestamps = data{:, 2};
plan_pos = data{:, 3:8};
actual_pos = data{:, 9:14};
current = data{:, 15:20};

% 计算相对时间（秒）- 时间戳现在是微秒单位
relative_time = (timestamps - timestamps(1)) / 1000000;

% 固定采样周期（秒）
dt = 0.005;

% 计算实际关节速度（度/秒）
actual_velocity = zeros(size(actual_pos));
for i = 2:size(actual_pos, 1)
    actual_velocity(i,:) = (actual_pos(i,:) - actual_pos(i-1,:)) / dt;
end

% 计算实际关节加速度（度/秒²）
actual_acceleration = zeros(size(actual_pos));
for i = 2:size(actual_velocity, 1)
    actual_acceleration(i,:) = (actual_velocity(i,:) - actual_velocity(i-1,:)) / dt;
end

% 计算计划关节速度（度/秒）
plan_velocity = zeros(size(plan_pos));
for i = 2:size(plan_pos, 1)
    plan_velocity(i,:) = (plan_pos(i,:) - plan_pos(i-1,:)) / dt;
end

% 计算计划关节加速度（度/秒²）
plan_acceleration = zeros(size(plan_pos));
for i = 2:size(plan_velocity, 1)
    plan_acceleration(i,:) = (plan_velocity(i,:) - plan_velocity(i-1,:)) / dt;
end

% 为了避免第一个点的速度和加速度为0，可以将第一个点设置为与第二个点相同
actual_velocity(1,:) = actual_velocity(2,:);
actual_acceleration(1,:) = actual_acceleration(2,:);
plan_velocity(1,:) = plan_velocity(2,:);
plan_acceleration(1,:) = plan_acceleration(2,:);

% 绘制位置图表
figure('Name', '关节位置', 'Position', [100, 100, 1200, 800]);
for i = 1:6
    subplot(3, 2, i);
    plot(relative_time, plan_pos(:, i), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(relative_time, actual_pos(:, i), 'r-', 'LineWidth', 1.5);
    title(['关节 ', num2str(i-1), ' 位置']);
    xlabel('时间 (秒)');
    ylabel('位置 (度)');
    legend('计划位置', '实际位置');
    grid on;
end

% 绘制速度图表
figure('Name', '关节速度', 'Position', [100, 100, 1200, 800]);
for i = 1:6
    subplot(3, 2, i);
    plot(relative_time, plan_velocity(:, i), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(relative_time, actual_velocity(:, i), 'r-', 'LineWidth', 1.5);
    title(['关节 ', num2str(i-1), ' 速度']);
    xlabel('时间 (秒)');
    ylabel('速度 (度/秒)');
    legend('计划速度', '实际速度');
    grid on;
end

% 绘制加速度图表
figure('Name', '关节加速度', 'Position', [100, 100, 1200, 800]);
for i = 1:6
    subplot(3, 2, i);
    plot(relative_time, plan_acceleration(:, i), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(relative_time, actual_acceleration(:, i), 'r-', 'LineWidth', 1.5);
    title(['关节 ', num2str(i-1), ' 加速度']);
    xlabel('时间 (秒)');
    ylabel('加速度 (度/秒²)');
    legend('计划加速度', '实际加速度');
    grid on;
end

% 绘制电流图表
figure('Name', '关节电流', 'Position', [100, 100, 1200, 800]);
for i = 1:6
    subplot(3, 2, i);
    plot(relative_time, current(:, i), 'g-', 'LineWidth', 1.5);
    title(['关节 ', num2str(i-1), ' 电流']);
    xlabel('时间 (秒)');
    ylabel('电流 (mA)');
    grid on;
end

fprintf('数据处理完成。共处理 %d 个数据点，时间范围 %.2f 秒\n', length(timestamps), relative_time(end));