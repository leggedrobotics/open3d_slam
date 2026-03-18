from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("open3d_slam_ros")
    default_param_dir = PathJoinSubstitution([package_share, "param"])
    default_map_dir = PathJoinSubstitution([package_share, "data", "maps"])

    return LaunchDescription([
        DeclareLaunchArgument("rosbag_filepath"),
        DeclareLaunchArgument("cloud_topic", default_value="/rslidar_points"),
        DeclareLaunchArgument("parameter_filename", default_value="param_robosense_rs16.yaml"),
        DeclareLaunchArgument("parameter_folder_path", default_value=default_param_dir),
        DeclareLaunchArgument("map_saving_folder", default_value=default_map_dir),
        DeclareLaunchArgument("num_accumulated_range_data", default_value="1"),
        Node(
            package="open3d_slam_ros",
            executable="mapping_node",
            name="open3d_slam",
            output="screen",
            parameters=[{
                "rosbag_filepath": LaunchConfiguration("rosbag_filepath"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "parameter_filename": LaunchConfiguration("parameter_filename"),
                "parameter_folder_path": LaunchConfiguration("parameter_folder_path"),
                "map_saving_folder": LaunchConfiguration("map_saving_folder"),
                "num_accumulated_range_data": LaunchConfiguration("num_accumulated_range_data"),
                "is_read_from_rosbag": True,
            }],
        ),
    ])
