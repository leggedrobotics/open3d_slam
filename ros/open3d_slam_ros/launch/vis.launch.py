import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
# 実行可否を指定するためのパッケージ
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    open3d_slam_ros_path = os.path.join(
        get_package_share_directory('open3d_slam_ros'))
    
    # Arguments
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=(open3d_slam_ros_path + '/rviz/default.rviz'))
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # RViz Node
    rviz_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config],
        condition=IfCondition(launch_rviz),
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        launch_rviz_arg,
        rviz_node,
    ])

