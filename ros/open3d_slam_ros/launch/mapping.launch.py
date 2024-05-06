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
    launch_prefix_arg = DeclareLaunchArgument('launch_prefix', default_value='')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value='/rslidar_points')
    parameter_filename_arg = DeclareLaunchArgument('parameter_filename', default_value='param_robosense_rs16.lua')
    parameter_folder_path_arg = DeclareLaunchArgument('parameter_folder_path', default_value=(open3d_slam_ros_path + '/param/'))
    map_saving_folder_arg = DeclareLaunchArgument('map_saving_folder', default_value=(open3d_slam_ros_path + '/data/maps/'))
    num_accumulated_range_data_arg = DeclareLaunchArgument('num_accumulated_range_data', default_value='1')
    is_read_from_rosbag_arg = DeclareLaunchArgument('is_read_from_rosbag', default_value='false')
    rosbag_filepath_arg = DeclareLaunchArgument('rosbag_filepath', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    launch_prefix = LaunchConfiguration('launch_prefix')
    launch_rviz = LaunchConfiguration('launch_rviz')
    cloud_topic = LaunchConfiguration('cloud_topic')
    parameter_filename = LaunchConfiguration('parameter_filename')
    parameter_folder_path = LaunchConfiguration('parameter_folder_path')
    map_saving_folder = LaunchConfiguration('map_saving_folder')
    num_accumulated_range_data = LaunchConfiguration('num_accumulated_range_data')
    is_read_from_rosbag = LaunchConfiguration('is_read_from_rosbag')
    rosbag_filepath = LaunchConfiguration('rosbag_filepath')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Main Node
    mapping_node = Node(
            package='open3d_slam_ros',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
            namespace='mapping',
            prefix=launch_prefix,
            parameters=[
                {
                    'cloud_topic': cloud_topic,
                    'parameter_folder_path': parameter_folder_path,
                    'parameter_filename': parameter_filename,
                    'num_accumulated_range_data': num_accumulated_range_data,
                    'is_read_from_rosbag': is_read_from_rosbag,
                    'rosbag_filepath': rosbag_filepath,
                    'map_saving_folder': map_saving_folder,
                }
            ]
        )

    # Visualization
    vis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(open3d_slam_ros_path + '/launch/vis.launch.py'),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            'cloud_topic': cloud_topic
        }.items()
    )

    return LaunchDescription([
        launch_prefix_arg,
        launch_rviz_arg,
        cloud_topic_arg,
        parameter_filename_arg,
        parameter_folder_path_arg,
        map_saving_folder_arg,
        num_accumulated_range_data_arg,
        is_read_from_rosbag_arg,
        rosbag_filepath_arg,
        use_sim_time_arg,
        mapping_node,
        vis_launch
    ])

