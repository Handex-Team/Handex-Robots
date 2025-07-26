clc; clear; close all;

% 设置 ROS2 变量
% setenv('ROS_DOMAIN_ID', '9');

% 创建 ROS2 节点
node = ros2node("matlab_odometry_node");

% 订阅 odom 话题
topicName = "/odom";
messageType = "nav_msgs/Odometry";

% 初始化全局变量
global xBuffer yBuffer zBuffer timeBuffer;
global vxBuffer axBuffer wzBuffer alphaBuffer;
xBuffer = []; yBuffer = []; zBuffer = []; timeBuffer = [];
vxBuffer = []; axBuffer = []; wzBuffer = []; alphaBuffer = [];

% 速度/角速度缓存（用于计算加速度）
global prev_vx prev_wz prev_time;
prev_vx = NaN; prev_wz = NaN; prev_time = NaN;

% 创建订阅者并附加回调函数
sub = ros2subscriber(node, topicName, messageType, @odometryCallback);

% 记录数据 10s
disp("开始订阅 /odom 话题...");
T_record = 10.0;
tic; % 计时
while toc < T_record
    drawnow; % 处理回调
    pause(0.1);
end
disp("数据采集完成，绘图中...");

%% **绘制 X-t, Y-t, Z-t 轨迹**
figure;
subplot(3,1,1);
plot(timeBuffer, xBuffer, 'b', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("X 位置 (m)");
title("X-t 轨迹"); grid on;

subplot(3,1,2);
plot(timeBuffer, yBuffer, 'r', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("Y 位置 (m)");
title("Y-t 轨迹"); grid on;

subplot(3,1,3);
plot(timeBuffer, zBuffer, 'g', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("Z 位置 (m)");
title("Z-t 轨迹"); grid on;

figure;
plot3(xBuffer, yBuffer,zBuffer, 'g', 'LineWidth', 2);
xlabel("X 位置 (m)"); ylabel("Y 位置 (m)");zlabel("Z 位置 (m)");
title("3D 轨迹"); grid on;

%% **绘制 速度、加速度、角速度、角加速度**
figure;
subplot(4,1,1);
plot(timeBuffer, vxBuffer, 'b', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("线速度 vx (m/s)");
title("vx-t 轨迹"); grid on;

subplot(4,1,2);
plot(timeBuffer, axBuffer, 'r', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("加速度 ax (m/s²)");
title("ax-t 轨迹"); grid on;

subplot(4,1,3);
plot(timeBuffer, wzBuffer, 'g', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("角速度 wz (rad/s)");
title("wz-t 轨迹"); grid on;

subplot(4,1,4);
plot(timeBuffer, alphaBuffer, 'm', 'LineWidth', 2);
xlabel("时间 (s)"); ylabel("角加速度 α (rad/s²)");
title("α-t 轨迹"); grid on;

disp("绘图完成。");

%% **回调函数**
function odometryCallback(msg)
    global xBuffer yBuffer zBuffer timeBuffer;
    global vxBuffer axBuffer wzBuffer alphaBuffer;
    global prev_vx prev_wz prev_time;
    
    % 获取当前时间
    t_now = toc; 
    
    % 提取位置信息
    pos = msg.pose.pose.position;
    
    % 提取线速度 & 角速度
    vx = msg.twist.twist.linear.x;
    wz = msg.twist.twist.angular.z;
    
    % 计算加速度 & 角加速度
    if ~isnan(prev_time)
        dt = t_now - prev_time;
        ax = (vx - prev_vx) / dt;
        alpha = (wz - prev_wz) / dt;
    else
        ax = 0; % 第一次数据点，加速度设为0
        alpha = 0;
    end
    
    % 存储数据
    xBuffer = [xBuffer; pos.x];
    yBuffer = [yBuffer; pos.y];
    zBuffer = [zBuffer; pos.z];
    timeBuffer = [timeBuffer; t_now];
    
    vxBuffer = [vxBuffer; vx];
    axBuffer = [axBuffer; ax];
    wzBuffer = [wzBuffer; wz];
    alphaBuffer = [alphaBuffer; alpha];
    
    % 更新上一次速度和时间
    prev_vx = vx;
    prev_wz = wz;
    prev_time = t_now;

    % 限制缓存大小
    maxBufferSize = 1000;
    if numel(xBuffer) > maxBufferSize
        xBuffer = xBuffer(end-maxBufferSize+1:end);
        yBuffer = yBuffer(end-maxBufferSize+1:end);
        zBuffer = zBuffer(end-maxBufferSize+1:end);
        timeBuffer = timeBuffer(end-maxBufferSize+1:end);
        vxBuffer = vxBuffer(end-maxBufferSize+1:end);
        axBuffer = axBuffer(end-maxBufferSize+1:end);
        wzBuffer = wzBuffer(end-maxBufferSize+1:end);
        alphaBuffer = alphaBuffer(end-maxBufferSize+1:end);
    end
end