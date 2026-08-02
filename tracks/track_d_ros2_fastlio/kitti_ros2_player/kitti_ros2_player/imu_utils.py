"""
Helpers for parsing KITTI Raw OXTS text records and turning them into
sensor_msgs/Imu messages.

Each line of oxts/data/XXXXXXXXXX.txt has 30 space-separated values.
Full field order is documented in oxts/dataformat.txt in the dataset;
the fields used here are:

  index  name    description
  3      roll    rad,  -pi..pi,  roll  (rotation around forward axis)
  4      pitch   rad,  -pi/2..pi/2, pitch (around left axis)
  5      yaw     rad,  -pi..pi,  heading (around up axis)
  11     ax      m/s^2, acceleration in forward direction
  12     ay      m/s^2, acceleration in left direction
  13     az      m/s^2, acceleration in up direction
  14     af      m/s^2, forward acceleration (bike/car frame, gravity compensated)
  15     al      m/s^2, leftward acceleration (bike/car frame, gravity compensated)
  16     au      m/s^2, upward acceleration (bike/car frame, gravity compensated)
  20     wf      rad/s, angular rate around forward axis
  21     wl      rad/s, angular rate around left axis
  22     wu      rad/s, angular rate around up axis

We publish orientation from roll/pitch/yaw, and angular_velocity /
linear_acceleration from the body-frame (forward/left/up) rates, which
is the convention FAST-LIO / FAST_LIO_GPU expect.
"""

import math
from sensor_msgs.msg import Imu

_FIELD_NAMES = [
    'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw',
    'vn', 've', 'vf', 'vl', 'vu',
    'ax', 'ay', 'az', 'af', 'al', 'au',
    'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
    'pos_accuracy', 'vel_accuracy',
    'navstat', 'numsats', 'posmode', 'velmode', 'orimode',
]


def parse_oxts_line(line: str) -> dict:
    """Parse one oxts data line into a dict keyed by field name."""
    values = [float(v) for v in line.strip().split()]
    if len(values) < len(_FIELD_NAMES):
        raise ValueError(
            f"Expected {len(_FIELD_NAMES)} oxts fields, got {len(values)}"
        )
    return dict(zip(_FIELD_NAMES, values))


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (radians, ZYX convention) to a quaternion (x, y, z, w)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def make_imu_msg(fields: dict, stamp, frame_id: str) -> Imu:
    """Build a sensor_msgs/Imu message from a parsed oxts field dict."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    qx, qy, qz, qw = euler_to_quaternion(fields['roll'], fields['pitch'], fields['yaw'])
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw

    msg.angular_velocity.x = fields['wf']
    msg.angular_velocity.y = fields['wl']
    msg.angular_velocity.z = fields['wu']

    msg.linear_acceleration.x = fields['af']
    msg.linear_acceleration.y = fields['al']
    msg.linear_acceleration.z = fields['au']

    return msg
