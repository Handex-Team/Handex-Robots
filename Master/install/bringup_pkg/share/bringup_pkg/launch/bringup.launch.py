import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import subprocess

ARGUMENTS = []
def generate_launch_description():


    ld = LaunchDescription(ARGUMENTS)
    
    path_find_node = Node(
        package='path_pkg',
        executable='stright_path',
        name='stright_path_node',
        output='screen'
    )

    traj_opt_node = Node(    
        package='traj_opt',
        executable='traj_opt',
        name='traj_opt_node',
        output='screen'
    )

    mpc_node = Node(
        package='mpc_pkg',
        executable='opt_traj_mpc',
        name='mpc_node',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('bringup_pkg'), 'rviz', 'adx_rviz.rviz')],
        output='screen'
    )



    ld.add_action(traj_opt_node)
    ld.add_action(path_find_node)
    ld.add_action(mpc_node)
    ld.add_action(rviz)

    return ld
