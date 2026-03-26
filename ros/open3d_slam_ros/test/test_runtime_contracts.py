import importlib.util
import math
import os
import signal
import struct
import subprocess
import threading
import time
import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


REPO_ROOT = Path(__file__).resolve().parents[3]
WORKSPACE_ROOT = Path(__file__).resolve().parents[5]
INSTALL_SETUP_PATH = WORKSPACE_ROOT / "install" / "setup.bash"
MAPPING_LAUNCH_PATH = REPO_ROOT / "ros" / "open3d_slam_ros" / "launch" / "mapping.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("open3d_slam_ros_mapping_launch", MAPPING_LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _make_launch_context(**launch_configurations):
    context = LaunchContext()
    context.launch_configurations.update({key: str(value) for key, value in launch_configurations.items()})
    return context


def _evaluate_mapping_launch_parameters(**overrides):
    defaults = dict(
        cloud_topic="/points",
        parameter_filename="param_robosense_rs16.yaml",
        parameter_folder_path="/tmp",
        map_saving_folder="/tmp/maps",
        num_accumulated_range_data="1",
        external_pose_frame="",
        external_pose_lookup_timeout_sec="0.1",
        publish_tf="",
    )
    defaults.update(overrides)
    context = _make_launch_context(**defaults)
    node = _load_launch_module()._launch_setup(context)[0]
    evaluated = evaluate_parameters(context, getattr(node, "_Node__parameters"))
    assert len(evaluated) == 1
    return evaluated[0]


def test_mapping_launch_omits_publish_tf_override_for_external_pose_mode():
    evaluated = _evaluate_mapping_launch_parameters(external_pose_frame="map")
    assert evaluated["external_pose_frame"] == "map"
    assert "publish_tf" not in evaluated


def test_mapping_launch_keeps_explicit_publish_tf_override():
    evaluated = _evaluate_mapping_launch_parameters(external_pose_frame="map", publish_tf="true")
    assert evaluated["publish_tf"] is True


class _LaunchProcess:
    def __init__(self, cloud_topic: str):
        self.cloud_topic = cloud_topic
        self.proc = None
        self.log_lines = []
        self._reader_thread = None

    def __enter__(self):
        command = (
            f"source {INSTALL_SETUP_PATH} && "
            "ros2 launch open3d_slam_ros mapping.launch.py "
            f"cloud_topic:={self.cloud_topic} "
            "external_pose_frame:=map "
            "parameter_filename:=param_robosense_rs16.yaml"
        )
        self.proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        self._reader_thread = threading.Thread(target=self._capture_logs, daemon=True)
        self._reader_thread.start()
        self.wait_for_log("publish_tf  set to: false", timeout_sec=20.0)
        self.wait_for_log("Created submap: 0", timeout_sec=20.0)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(timeout=5.0)

    def _capture_logs(self):
        assert self.proc is not None
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            self.log_lines.append(line.rstrip())

    def wait_for_log(self, fragment: str, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if any(fragment in line for line in self.log_lines):
                return
            if self.proc is not None and self.proc.poll() is not None:
                raise AssertionError(
                    f"launch exited before logging {fragment!r}\n" + "\n".join(self.log_lines[-40:])
                )
            time.sleep(0.05)
        raise AssertionError(f"timed out waiting for {fragment!r}\n" + "\n".join(self.log_lines[-40:]))


class _DelayedTfProbe(Node):
    def __init__(self, cloud_topic: str, frame_id: str):
        super().__init__("open3d_slam_delayed_tf_probe")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.create_subscription(Odometry, "/scan2scan_odometry", self._scan2scan_cb, qos)
        self.create_subscription(Odometry, "/scan2map_odometry", self._scan2map_cb, qos)
        self.create_subscription(PointCloud2, "/assembled_map", self._assembled_cb, qos)
        self.frame_id = frame_id
        self.tf_broadcaster = None
        self.publish_count = 0
        self.scan2scan_msg = None
        self.scan2map_msg = None
        self.assembled_msg = None
        self.scan2scan_first_publish_count = None

    def _scan2scan_cb(self, msg):
        if self.scan2scan_first_publish_count is None:
            self.scan2scan_first_publish_count = self.publish_count
        self.scan2scan_msg = msg

    def _scan2map_cb(self, msg):
        self.scan2map_msg = msg

    def _assembled_cb(self, msg):
        self.assembled_msg = msg

    def enable_static_tf(self):
        if self.tf_broadcaster is None:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = self.frame_id
        msg.transform.translation.x = 1.0
        msg.transform.translation.y = -0.5
        msg.transform.translation.z = 0.25
        msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(msg)

    def make_cloud(self):
        points = []
        for i in range(16):
            for j in range(16):
                x = 4.0 + 0.25 * i
                y = -2.0 + 0.25 * j
                z = -0.3 + 0.12 * math.sin(0.4 * x) + 0.08 * math.cos(0.6 * y)
                points.append((x, y, z))
        for k in range(48):
            angle = 2.0 * math.pi * k / 48.0
            points.append((6.5 + 0.6 * math.cos(angle), 1.2 + 0.5 * math.sin(angle), 0.6 + 0.05 * math.sin(2.0 * angle)))
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b"".join(struct.pack("fff", *point) for point in points)
        return msg


def test_external_pose_mode_keeps_icp_odometry_during_tf_gap_and_resumes_map_outputs():
    suffix = uuid.uuid4().hex[:8]
    cloud_topic = f"/test_cloud_{suffix}"
    frame_id = f"test_lidar_{suffix}"

    with _LaunchProcess(cloud_topic):
        node = None
        rclpy.init()
        try:
            node = _DelayedTfProbe(cloud_topic, frame_id)
            enable_tf_at = 60
            deadline = time.time() + 50.0
            while time.time() < deadline:
                if node.publish_count >= enable_tf_at:
                    node.enable_static_tf()
                node.cloud_pub.publish(node.make_cloud())
                node.publish_count += 1

                slice_deadline = time.time() + 0.2
                while time.time() < slice_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)

                if node.scan2scan_msg and node.publish_count >= enable_tf_at and node.scan2map_msg and node.assembled_msg:
                    break

            assert node.scan2scan_msg is not None
            assert node.scan2scan_first_publish_count is not None
            assert node.scan2scan_first_publish_count < enable_tf_at
            assert node.scan2scan_msg.header.frame_id == "odom_o3d"
            assert node.scan2map_msg is not None
            assert node.scan2map_msg.header.frame_id == "map"
            assert node.assembled_msg is not None
            assert node.assembled_msg.header.frame_id == "map"
            assert node.assembled_msg.width * node.assembled_msg.height > 0
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
