from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_optional_bool(value: str, arg_name: str):
    if value == "":
        return None
    if value not in {"true", "false"}:
        raise RuntimeError(f"{arg_name} must be one of: true, false, or empty")
    return value == "true"


def _launch_setup(context, *args, **kwargs):
    params = {
        "cloud_topic": LaunchConfiguration("cloud_topic").perform(context),
        "parameter_filename": LaunchConfiguration("parameter_filename").perform(context),
        "parameter_folder_path": LaunchConfiguration("parameter_folder_path").perform(context),
        "map_saving_folder": LaunchConfiguration("map_saving_folder").perform(context),
        "num_accumulated_range_data": int(LaunchConfiguration("num_accumulated_range_data").perform(context)),
        "external_pose_frame": LaunchConfiguration("external_pose_frame").perform(context),
        "external_pose_lookup_timeout_sec": float(LaunchConfiguration("external_pose_lookup_timeout_sec").perform(context)),
        "is_read_from_rosbag": False,
    }
    publish_tf = _resolve_optional_bool(LaunchConfiguration("publish_tf").perform(context), "publish_tf")
    if publish_tf is not None:
        params["publish_tf"] = publish_tf

    return [
        Node(
            package="open3d_slam_ros",
            executable="mapping_node",
            name="open3d_slam",
            output="screen",
            parameters=[params],
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("open3d_slam_ros")
    default_param_dir = PathJoinSubstitution([package_share, "param"])
    default_map_dir = PathJoinSubstitution([package_share, "data", "maps"])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/rslidar_points"),
        DeclareLaunchArgument("parameter_filename", default_value="param_robosense_rs16.yaml"),
        DeclareLaunchArgument("parameter_folder_path", default_value=default_param_dir),
        DeclareLaunchArgument("map_saving_folder", default_value=default_map_dir),
        DeclareLaunchArgument("num_accumulated_range_data", default_value="1"),
        DeclareLaunchArgument("external_pose_frame", default_value=""),
        DeclareLaunchArgument("external_pose_lookup_timeout_sec", default_value="0.1"),
        DeclareLaunchArgument("publish_tf", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
