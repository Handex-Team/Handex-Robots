#!/usr/bin/env python3
####还有问题没有解决，用不了
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    # 公共参数
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # 第一个雷达参数
    serial_port1 = LaunchConfiguration('serial_port1', default='/dev/ttyUSB0')
    frame_id1 = LaunchConfiguration('frame_id1', default='laser')

    # 第二个雷达参数
    serial_port2 = LaunchConfiguration('serial_port2', default='/dev/ttyUSB1')
    frame_id2 = LaunchConfiguration('frame_id2', default='base_link')

    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Channel type for lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Serial baudrate'),

        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Angle compensate'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description='Scan mode'),

        DeclareLaunchArgument(
            'serial_port1',
            default_value='/dev/ttyUSB0',
            description='Serial port for lidar 1'),

        DeclareLaunchArgument(
            'frame_id1',
            default_value='laser1',
            description='Frame ID for lidar 1'),

        DeclareLaunchArgument(
            'serial_port2',
            default_value='/dev/ttyUSB1',
            description='Serial port for lidar 2'),

        DeclareLaunchArgument(
            'frame_id2',
            default_value='laser2',
            description='Frame ID for lidar 2'),

        # 启动第一个雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_1',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port1,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id1,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
            output='screen'),

        # 启动第二个雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port2,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id2,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
            output='screen'),
    ])
