import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import subprocess

# 声明启动参数
ARGUMENTS = [
    DeclareLaunchArgument('GOAL_X', default_value='0.0',
                          description='目标点x坐标'),
    DeclareLaunchArgument('GOAL_Y', default_value='0.0',
                          description='目标点y坐标')
]

my_pkg_share_dir = get_package_share_directory('my_pkg')
map_pkg_share_dir = get_package_share_directory('map_pkg')


# Declare launch configuration variables that can access the launch arguments values
visualization_config_file = LaunchConfiguration('visualization_config')
rviz_config_file = LaunchConfiguration('rviz_config')

# 读取参数
map_parameters = os.path.join(map_pkg_share_dir, 'params', 'map.yaml')

def generate_launch_description():


    # Declare launch arguments
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            my_pkg_share_dir, 'config', 'grid_map.yaml'),
        description='可视化配置文件的路径')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            my_pkg_share_dir, 'rviz', 'my_rviz.rviz'),
        description='rviz2配置文件的路径')
    
    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    #####################################################################

    # 激光节点
    laser = Node(
        package='my_pkg',
        executable='laser',
        name='laser_node',
        output='screen',
        parameters=[]
    )
    
    # APF路径节点
    map_generator = Node(
        package='map_pkg',
        executable='grid_map_generator',
        name='grid_map_generator_node',
        output='screen',
        parameters=[map_parameters]
    )

    apf_path = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='my_pkg',
                executable='grid_apf',
                name='grid_apf_path_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # MPC节点
    mpc = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='my_pkg',
                executable='mpc',
                name='mpc_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    real_state = Node(
        package='my_pkg',
        executable='real_state',
        name='real_state_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 可视化节点
    vis = Node(
        package='my_pkg',
        executable='vis',
        name='vis_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    A_star=Node(
        package='my_pkg',
        executable='A_star',
        name='A_star_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rrt_star=Node(
        package='rrt_star',
        executable='rrt_star',
        name='rrt_star_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 创建 LaunchDescription
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(laser)
    ld.add_action(apf_path)
    ld.add_action(rrt_star)
    # ld.add_action(mpc)
    ld.add_action(vis)
    ld.add_action(real_state)
    ld.add_action(map_generator)
    ld.add_action(A_star)
    

    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)

    return ld
