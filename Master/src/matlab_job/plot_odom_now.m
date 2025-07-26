% odom_cmd_vel_visualizer.m - 实时可视化 odom 和 cmd_vel 数据

clear;
clc;
close all;

% 初始化 ROS2 节点
node = ros2node("/odom_cmd_vel_visualizer_node");

% 创建订阅器
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
cmdVelSub = ros2subscriber(node, "/cmd_vel", "geometry_msgs/Twist");

% 创建新图形窗口
figure('Name', 'Odom 和 Cmd_Vel 可视化', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);

% 轨迹图
subplot(3,3,[1,2]);
grid on; title('机器人轨迹'); xlabel('X (m)'); ylabel('Y (m)');
hold on;
trajPlot = animatedline('Color', 'b', 'LineWidth', 2);
robotMarker = plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
axis equal;

% Odom 速度
subplot(3,3,3); grid on; title('Odom 线速度 (vx)'); xlabel('时间 (s)'); ylabel('m/s');
odomVxPlot = animatedline('Color', 'g', 'DisplayName', '实际速度');
hold on;
cmdVxPlot = animatedline('Color', 'b', 'DisplayName', '指令速度');
legend('Location', 'best');

subplot(3,3,6); grid on; title('Odom 角速度 (wz)'); xlabel('时间 (s)'); ylabel('rad/s');
odomWzPlot = animatedline('Color', 'm', 'DisplayName', '实际角速度');
hold on;
cmdWzPlot = animatedline('Color', 'c', 'DisplayName', '指令角速度');
legend('Location', 'best');

% Cmd_vel 指令速度
subplot(3,3,4); grid on; title('Cmd_Vel 线速度指令'); xlabel('时间 (s)'); ylabel('m/s');
cmdVxPlot2 = animatedline('Color', 'b', 'LineWidth', 1.5);

subplot(3,3,7); grid on; title('Cmd_Vel 角速度指令'); xlabel('时间 (s)'); ylabel('rad/s');
cmdWzPlot2 = animatedline('Color', 'c', 'LineWidth', 1.5);

% 加速度
subplot(3,3,5); grid on; title('线加速度 (ax)'); xlabel('时间 (s)'); ylabel('m/s²');
axPlot = animatedline('Color', 'r');

subplot(3,3,8); grid on; title('角加速度 (az)'); xlabel('时间 (s)'); ylabel('rad/s²');
azPlot = animatedline('Color', 'k');

% 初始化变量
prev_time = [];
prev_vx = 0;
prev_wz = 0;
start_time = nan;

disp("开始接收数据...");
while true
    try
        % 获取最新消息
        [odomMsg, odomStatus] = receive(odomSub, 1);  % 最多等待1秒
        [cmdVelMsg, cmdVelStatus] = receive(cmdVelSub, 0.1);  % 非阻塞接收
        
        % 检查是否收到新消息
        if ~odomStatus
            continue;
        end
        
        % 时间处理
        t = double(odomMsg.header.stamp.sec) + double(odomMsg.header.stamp.nanosec) * 1e-9;
        
        % 如果是第一次迭代，设置起始时间
        if isnan(start_time)
            start_time = t;
        end
        relative_time = t - start_time;
        
        % 位置
        x = odomMsg.pose.pose.position.x;
        y = odomMsg.pose.pose.position.y;
        
        % 速度
        vx = odomMsg.twist.twist.linear.x;
        wz = odomMsg.twist.twist.angular.z;
        
        % 加速度（数值微分）
        if isempty(prev_time)
            ax = 0;
            az = 0;
        else
            dt = t - prev_time;
            if dt > 0
                ax = (vx - prev_vx) / dt;
                az = (wz - prev_wz) / dt;
            else
                ax = 0;
                az = 0;
            end
        end
        
        % 保存当前值
        prev_time = t;
        prev_vx = vx;
        prev_wz = wz;
        
        % 更新轨迹图
        subplot(3,3,[1,2]);
        addpoints(trajPlot, x, y);
        set(robotMarker, 'XData', x, 'YData', y);
        
        % 更新速度比较图
        subplot(3,3,3);
        addpoints(odomVxPlot, relative_time, vx);
        
        subplot(3,3,6);
        addpoints(odomWzPlot, relative_time, wz);
        
        % 更新加速度图
        subplot(3,3,5);
        addpoints(axPlot, relative_time, ax);
        
        subplot(3,3,8);
        addpoints(azPlot, relative_time, az);
        
        % 如果有 cmd_vel 数据，更新指令速度图
        if cmdVelStatus
            cmd_vx = cmdVelMsg.linear.x;
            cmd_wz = cmdVelMsg.angular.z;
            
            subplot(3,3,3);
            addpoints(cmdVxPlot, relative_time, cmd_vx);
            
            subplot(3,3,6);
            addpoints(cmdWzPlot, relative_time, cmd_wz);
            
            subplot(3,3,4);
            addpoints(cmdVxPlot2, relative_time, cmd_vx);
            
            subplot(3,3,7);
            addpoints(cmdWzPlot2, relative_time, cmd_wz);
        end
        
        drawnow limitrate;
        
    catch ME
        % 处理可能的错误（如窗口关闭）
        if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
            disp('可视化窗口已关闭，退出程序...');
            break;
        else
            rethrow(ME);
        end
    end
end

% 清理
clear node;
disp('程序结束');