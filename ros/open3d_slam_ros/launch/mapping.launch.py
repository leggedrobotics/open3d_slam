from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    parameter_filename = LaunchConfiguration("parameter_filename")
    parameter_folder_path = LaunchConfiguration("parameter_folder_path")
    map_saving_folder = LaunchConfiguration("map_saving_folder")
    num_accumulated_range_data = LaunchConfiguration("num_accumulated_range_data")
    is_read_from_rosbag = LaunchConfiguration("is_read_from_rosbag")
    rosbag_filepath = LaunchConfiguration("rosbag_filepath")

    rviz_config = PathJoinSubstitution([FindPackageShare("open3d_slam_ros"), "rviz", "default.rviz"])

    mapping_node = Node(
        package="open3d_slam_ros",
        executable="mapping_node",
        name="open3d_slam",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"cloud_topic": cloud_topic},
            {"parameter_folder_path": parameter_folder_path},
            {"parameter_filename": parameter_filename},
            {"num_accumulated_range_data": ParameterValue(num_accumulated_range_data, value_type=int)},
            {"is_read_from_rosbag": ParameterValue(is_read_from_rosbag, value_type=bool)},
            {"rosbag_filepath": rosbag_filepath},
            {"map_saving_folder": map_saving_folder},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="open3d_slam_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(launch_rviz),
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
            DeclareLaunchArgument("num_accumulated_range_data", default_value="1"),
            DeclareLaunchArgument("is_read_from_rosbag", default_value="false"),
            DeclareLaunchArgument("rosbag_filepath", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            mapping_node,
            rviz_node,
        ]
    )
