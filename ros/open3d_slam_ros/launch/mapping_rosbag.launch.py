from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    cloud_topic = LaunchConfiguration("cloud_topic")
    launch_rviz = LaunchConfiguration("launch_rviz")
    parameter_filename = LaunchConfiguration("parameter_filename")
    parameter_folder_path = LaunchConfiguration("parameter_folder_path")
    map_saving_folder = LaunchConfiguration("map_saving_folder")
    is_read_from_rosbag = LaunchConfiguration("is_read_from_rosbag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    play_delay = LaunchConfiguration("play_delay")
    play_rate = LaunchConfiguration("play_rate")
    bag_filename = LaunchConfiguration("bag_filename")
    bag_folder_path = LaunchConfiguration("bag_folder_path")
    rosbag_full_path = PathJoinSubstitution([bag_folder_path, bag_filename])

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("open3d_slam_ros"), "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
            "cloud_topic": cloud_topic,
            "launch_rviz": launch_rviz,
            "parameter_filename": parameter_filename,
            "parameter_folder_path": parameter_folder_path,
            "num_accumulated_range_data": "1",
            "use_sim_time": use_sim_time,
            "is_read_from_rosbag": is_read_from_rosbag,
            "rosbag_filepath": rosbag_full_path,
            "map_saving_folder": map_saving_folder,
        }.items(),
    )

    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            rosbag_full_path,
            "--delay",
            play_delay,
            "--rate",
            play_rate,
            "--clock",
        ],
        output="screen",
        condition=UnlessCondition(is_read_from_rosbag),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("parameter_filename", default_value="param_robosense_rs16.lua"),
            DeclareLaunchArgument(
                "parameter_folder_path",
                default_value=PathJoinSubstitution([FindPackageShare("open3d_slam_ros"), "param"]),
            ),
            DeclareLaunchArgument(
                "map_saving_folder",
                default_value=PathJoinSubstitution([FindPackageShare("open3d_slam_ros"), "data", "maps"]),
            ),
            DeclareLaunchArgument("is_read_from_rosbag", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("play_delay", default_value="0.4"),
            DeclareLaunchArgument("play_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_filename", default_value="wheeled_robot_large_outdoor_area.bag"),
            DeclareLaunchArgument(
                "bag_folder_path",
                default_value=PathJoinSubstitution([FindPackageShare("open3d_slam_ros"), "data"]),
            ),
            mapping_launch,
            rosbag_play,
        ]
    )
