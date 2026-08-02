"""
kitti_player.py

Plays back a single KITTI Raw "drive" folder (e.g. 2011_10_03_drive_0047_sync)
as ROS 2 topics:

    /velodyne_points   (sensor_msgs/PointCloud2)
    /imu/data          (sensor_msgs/Imu)
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, Imu

from kitti_ros2_player.timestamp_utils import load_timestamps
from kitti_ros2_player.pointcloud_utils import load_velodyne_bin, make_pointcloud2_with_ring_time
from kitti_ros2_player.imu_utils import parse_oxts_line, make_imu_msg


class KittiPlayer(Node):

    def __init__(self):
        super().__init__('kitti_player')

        self.declare_parameter('dataset_path', '')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('lidar_frame_id', 'velodyne')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('tick_period', 0.002)
        self.declare_parameter('scan_lines', 32)   # must match FAST-LIO's scan_line config
        self.declare_parameter('scan_rate_hz', 10.0)  # Velodyne rotation rate

        self.dataset_path = self._get_str('dataset_path')
        self.playback_rate = self._get_float('playback_rate')
        self.loop = self._get_bool('loop')
        self.lidar_frame_id = self._get_str('lidar_frame_id')
        self.imu_frame_id = self._get_str('imu_frame_id')
        lidar_topic = self._get_str('lidar_topic')
        imu_topic = self._get_str('imu_topic')
        self.start_delay = self._get_float('start_delay')
        tick_period = self._get_float('tick_period')
        self.scan_lines = self.get_parameter('scan_lines').get_parameter_value().integer_value
        scan_rate_hz = self._get_float('scan_rate_hz')
        self.scan_period = 1.0 / scan_rate_hz if scan_rate_hz > 0 else 0.1

        if not self.dataset_path:
            self.get_logger().error(
                "Parameter 'dataset_path' is required, e.g. "
                "/data/kitti/2011_10_03/2011_10_03_drive_0047_sync"
            )
            raise SystemExit(1)

        self.lidar_pub = self.create_publisher(PointCloud2, lidar_topic, 50)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 200)

        self._events = []
        self._num_lidar = 0
        self._num_imu = 0
        self._load_dataset()

        self._cursor = 0
        self._wall_start = None
        self._sim_start = self.get_clock().now()

        self.get_logger().info(
            f"Loaded {len(self._events)} events "
            f"({self._num_lidar} lidar, {self._num_imu} imu) from {self.dataset_path}"
        )
        self.get_logger().info(
            f"Playback rate: {self.playback_rate}x, starting in {self.start_delay:.1f}s"
        )

        self._timer = self.create_timer(tick_period, self._on_tick)

    # -- parameter helpers -------------------------------------------------

    def _get_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_bool(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    # -- dataset loading ------------------------------------------------

    def _load_dataset(self):
        velodyne_dir = os.path.join(self.dataset_path, 'velodyne_points')
        oxts_dir = os.path.join(self.dataset_path, 'oxts')

        velodyne_ts_path = os.path.join(velodyne_dir, 'timestamps.txt')
        oxts_ts_path = os.path.join(oxts_dir, 'timestamps.txt')

        if not os.path.isfile(velodyne_ts_path):
            raise FileNotFoundError(f"Missing {velodyne_ts_path}")
        if not os.path.isfile(oxts_ts_path):
            raise FileNotFoundError(f"Missing {oxts_ts_path}")

        lidar_times = load_timestamps(velodyne_ts_path)
        imu_times = load_timestamps(oxts_ts_path)

        lidar_bin_dir = os.path.join(velodyne_dir, 'data')
        oxts_data_dir = os.path.join(oxts_dir, 'data')

        lidar_files = sorted(os.listdir(lidar_bin_dir))
        oxts_files = sorted(os.listdir(oxts_data_dir))

        if len(lidar_files) != len(lidar_times):
            self.get_logger().warn(
                f"Velodyne file count ({len(lidar_files)}) != "
                f"timestamp count ({len(lidar_times)}); truncating to the shorter."
            )
        if len(oxts_files) != len(imu_times):
            self.get_logger().warn(
                f"OXTS file count ({len(oxts_files)}) != "
                f"timestamp count ({len(imu_times)}); truncating to the shorter."
            )

        n_lidar = min(len(lidar_files), len(lidar_times))
        n_imu = min(len(oxts_files), len(imu_times))

        events = []
        for i in range(n_lidar):
            events.append((lidar_times[i], 'lidar', os.path.join(lidar_bin_dir, lidar_files[i])))
        for i in range(n_imu):
            events.append((imu_times[i], 'imu', os.path.join(oxts_data_dir, oxts_files[i])))

        events.sort(key=lambda e: e[0])

        self._events = events
        self._num_lidar = n_lidar
        self._num_imu = n_imu

    # -- playback ------------------------------------------------------

    def _on_tick(self):
        now = time.monotonic()

        if self._wall_start is None:
            self._wall_start = now
            return

        elapsed_wall = now - self._wall_start
        if elapsed_wall < self.start_delay:
            return

        sim_elapsed = (elapsed_wall - self.start_delay) * self.playback_rate

        while self._cursor < len(self._events) and self._events[self._cursor][0] <= sim_elapsed:
            t_rel, kind, path = self._events[self._cursor]
            stamp = (self._sim_start + Duration(seconds=t_rel)).to_msg()

            try:
                if kind == 'lidar':
                    points = load_velodyne_bin(path)
                    msg = make_pointcloud2_with_ring_time(
                        points, stamp, self.lidar_frame_id,
                        n_scans=self.scan_lines, scan_period=self.scan_period,
                    )
                    self.lidar_pub.publish(msg)
                else:
                    with open(path, 'r') as f:
                        line = f.readline()
                    fields = parse_oxts_line(line)
                    msg = make_imu_msg(fields, stamp, self.imu_frame_id)
                    self.imu_pub.publish(msg)
            except Exception as exc:  # noqa: BLE001 - keep playback going on a bad frame
                self.get_logger().warn(f"Failed to publish {kind} event ({path}): {exc}")

            self._cursor += 1

        if self._cursor >= len(self._events):
            if self.loop:
                self.get_logger().info("Reached end of dataset, looping.")
                self._cursor = 0
                self._wall_start = None
                self._sim_start = self.get_clock().now()
            else:
                self.get_logger().info("Reached end of dataset. Shutting down.")
                self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = KittiPlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
