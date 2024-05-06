import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# 実行可否を指定するためのパッケージ
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter
def generate_launch_description():
    open3d_slam_ros_path = os.path.join(
        get_package_share_directory('open3d_slam_ros'))

    # Arguments
    launch_prefix_arg = DeclareLaunchArgument('launch_prefix', default_value='') #'gdb -ex run --args')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value='/rslidar_points')
    parameter_filename_arg = DeclareLaunchArgument('parameter_filename', default_value='param_robosense_rs16.lua')
    parameter_folder_path_arg = DeclareLaunchArgument('parameter_folder_path', default_value=open3d_slam_ros_path + '/param/')
    num_accumulated_range_data_arg = DeclareLaunchArgument('num_accumulated_range_data', default_value='1')
    is_read_from_rosbag_arg = DeclareLaunchArgument('is_read_from_rosbag', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    play_delay_arg = DeclareLaunchArgument('play_delay', default_value='0.4')
    play_rate_arg = DeclareLaunchArgument('play_rate', default_value='1.0')
    bag_filename_arg = DeclareLaunchArgument('bag_filename', default_value='wheeled_robot_large_outdoor_area')
    bag_folder_path_arg = DeclareLaunchArgument('bag_folder_path', default_value=open3d_slam_ros_path + '/data/')

    launch_prefix = LaunchConfiguration('launch_prefix')
    launch_rviz = LaunchConfiguration('launch_rviz')
    cloud_topic = LaunchConfiguration('cloud_topic')
    parameter_filename = LaunchConfiguration('parameter_filename')
    parameter_folder_path = LaunchConfiguration('parameter_folder_path')
    num_accumulated_range_data = LaunchConfiguration('num_accumulated_range_data')
    is_read_from_rosbag = LaunchConfiguration('is_read_from_rosbag')
    use_sim_time = LaunchConfiguration('use_sim_time')
    play_delay = LaunchConfiguration('play_delay')
    play_rate = LaunchConfiguration('play_rate')
    bag_filename = LaunchConfiguration('bag_filename')
    bag_folder_path = LaunchConfiguration('bag_folder_path')

    # Mapping Launch File
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(open3d_slam_ros_path + '/launch/mapping.launch.py'),
        launch_arguments={
            'cloud_topic': cloud_topic,
            'launch_rviz': launch_rviz,
            'parameter_filename': parameter_filename,
            'parameter_folder_path': parameter_folder_path,
            'num_accumulated_range_data': num_accumulated_range_data,
            'use_sim_time': use_sim_time,
            'is_read_from_rosbag': is_read_from_rosbag,
            'rosbag_filepath': PathJoinSubstitution([bag_folder_path, bag_filename]),
            'launch_prefix': launch_prefix
        }.items()
    )

    # Play Rosbag Node
    play_rosbag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', PathJoinSubstitution([bag_folder_path, bag_filename]), '--delay', play_delay, '--rate', play_rate, '--clock'],
        condition=UnlessCondition(is_read_from_rosbag),
        output='screen'
    )
    # append --loop for looping

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        launch_prefix_arg,
        launch_rviz_arg,
        cloud_topic_arg,
        parameter_filename_arg,
        parameter_folder_path_arg,
        num_accumulated_range_data_arg,
        is_read_from_rosbag_arg,
        play_delay_arg,
        play_rate_arg,
        bag_filename_arg,
        bag_folder_path_arg,
        use_sim_time_arg,
        mapping_launch,
        play_rosbag_node
    ])

