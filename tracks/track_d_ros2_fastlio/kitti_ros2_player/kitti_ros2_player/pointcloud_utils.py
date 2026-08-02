"""
Helpers for loading KITTI Raw Velodyne .bin scans and turning them into
sensor_msgs/PointCloud2 messages.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def load_velodyne_bin(bin_path: str) -> np.ndarray:
    """
    Load a single KITTI Velodyne .bin file.

    Each point is 4 x float32: (x, y, z, reflectance/intensity).
    Returns an (N, 4) float32 array.
    """
    scan = np.fromfile(bin_path, dtype=np.float32)
    return scan.reshape(-1, 4)


def make_pointcloud2(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    """
    Build a plain sensor_msgs/PointCloud2 message from an (N, 4) float32
    array of (x, y, z, intensity) points. No `ring`/`time` fields --
    use make_pointcloud2_with_ring_time() for FAST-LIO's Velodyne
    preprocessing, which requires those fields.
    """
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = points.shape[0]

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields * 4 bytes
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()

    return msg


def compute_ring_and_time(points: np.ndarray, n_scans: int = 32, scan_period: float = 0.1):
    """
    Approximate per-point `ring` (beam index) and `time` (microseconds
    within the scan) for a KITTI Velodyne scan, since the raw .bin files
    don't carry either.

    - ring: bucketed from each point's vertical angle into n_scans bins
      spanning this scan's actual min/max vertical angle. Only an
      approximation of the sensor's true beam layout, but adequate when
      feature_extract_enable is false (ring isn't used for feature
      classification in that mode).
    - time: linear in azimuth angle across the scan's `scan_period`
      (KITTI's Velodyne spins at 10 Hz, i.e. scan_period=0.1s), in
      microseconds, matching `timestamp_unit: 2` in FAST-LIO's
      velodyne.yaml.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    horiz_dist = np.sqrt(x * x + y * y)
    vertical_angle = np.degrees(np.arctan2(z, np.maximum(horiz_dist, 1e-6)))

    min_angle = float(vertical_angle.min())
    max_angle = float(vertical_angle.max())
    angle_range = max(max_angle - min_angle, 1e-6)
    ring = np.round((vertical_angle - min_angle) / angle_range * (n_scans - 1))
    ring = np.clip(ring, 0, n_scans - 1).astype(np.uint16)

    azimuth = np.arctan2(y, x)          # -pi .. pi
    azimuth = np.mod(-azimuth, 2 * np.pi)  # 0 .. 2pi, monotonic sweep direction
    time_us = (azimuth / (2 * np.pi) * scan_period * 1e6).astype(np.float32)

    return ring, time_us


def make_pointcloud2_with_ring_time(
    points: np.ndarray, stamp, frame_id: str, n_scans: int = 32, scan_period: float = 0.1
) -> PointCloud2:
    """
    Build a sensor_msgs/PointCloud2 with x, y, z, intensity, time, ring --
    the fields FAST-LIO's velodyne_ros::Point preprocessing expects
    (matched by name via pcl_conversions, so our on-wire byte layout
    doesn't need to match FAST-LIO's internal C++ struct layout).
    """
    ring, time_us = compute_ring_and_time(points, n_scans=n_scans, scan_period=scan_period)
    n = points.shape[0]

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 24  # 20 bytes of fields + 2 (ring) + 2 padding, 4-byte aligned
    msg.row_step = msg.point_step * n
    msg.is_dense = True

    buf = np.zeros(
        n,
        dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('time', np.float32),
            ('ring', np.uint16), ('_pad', np.uint16),
        ],
    )
    buf['x'] = points[:, 0]
    buf['y'] = points[:, 1]
    buf['z'] = points[:, 2]
    buf['intensity'] = points[:, 3]
    buf['time'] = time_us
    buf['ring'] = ring

    msg.data = buf.tobytes()
    return msg
