# kitti_ros2_player

A ROS 2 node that replays a KITTI **Raw** drive (e.g. `2011_10_03_drive_0047_sync`)
as `/velodyne_points` (`sensor_msgs/PointCloud2`) and `/imu/data`
(`sensor_msgs/Imu`), for feeding into FAST-LIO / FAST_LIO_GPU.

## Expected dataset layout

```
2011_10_03_drive_0047_sync/
    velodyne_points/
        timestamps.txt
        data/
            0000000000.bin
            0000000001.bin
            ...
    oxts/
        timestamps.txt
        data/
            0000000000.txt
            0000000001.txt
            ...
```

This is the standard "synced+rectified" KITTI Raw download structure. No
conversion to the KITTI Odometry benchmark format is needed.

---

## Terminal 1 — build the package

```bash
mkdir -p ~/fastlio2_ws/src
cd ~/fastlio2_ws/src
# copy (or unzip) the kitti_ros2_player folder here so you have:
#   ~/fastlio2_ws/src/kitti_ros2_player/

cd ~/fastlio2_ws
colcon build --packages-select kitti_ros2_player
source install/setup.bash
```

## Terminal 1 (continued) — set your dataset path and run

Edit `config/params.yaml` (or pass the path on the command line) to point
at your drive folder, then launch:

```bash
ros2 launch kitti_ros2_player player.launch.py \
  params_file:=/home/$USER/fastlio2_ws/src/kitti_ros2_player/config/params.yaml
```

Or override just the dataset path without editing the file:

```bash
ros2 run kitti_ros2_player kitti_player --ros-args \
  -p dataset_path:=/home/$USER/data/kitti/2011_10_03/2011_10_03_drive_0047_sync \
  -p playback_rate:=1.0
```

You should see log lines like:

```
[kitti_player]: Loaded 1085 events (413 lidar, 672 imu) from /home/.../2011_10_03_drive_0047_sync
[kitti_player]: Playback rate: 1.0x, starting in 1.0s
```

## Terminal 2 — verify topics are publishing

```bash
source ~/fastlio2_ws/install/setup.bash
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
```

You should see roughly 10 Hz on `/velodyne_points` and 100 Hz on
`/imu/data` (KITTI's native rates) once playback starts.

## Terminal 3 — run FAST-LIO / FAST_LIO_GPU

```bash
source ~/fastlio2_ws/install/setup.bash
ros2 launch fast_lio_gpu mapping.launch.py   # adjust to your actual launch file name
```

Make sure FAST-LIO's config (`pointcloud_topic`, `imu_topic`, and the
lidar/imu extrinsics) matches the topic names and frame IDs set in
`config/params.yaml` (`/velodyne_points`, `/imu/data`, `velodyne`,
`imu_link` by default).

---

## Notes / things to double-check

- **Timestamps are relative, not absolute.** Message stamps are built from
  the node's start time plus the dataset's relative offsets (parsed from
  `timestamps.txt`), not the original 2011 wall-clock date. Relative
  ordering and spacing between lidar and IMU messages is preserved, which
  is what FAST-LIO needs — but don't rely on the stamps for anything that
  needs true UTC time.
- **IMU axis convention.** `angular_velocity` and `linear_acceleration`
  are published using OXTS's body-frame (forward, left, up) rates (`wf`,
  `wl`, `wu` / `af`, `al`, `au`). If FAST-LIO's extrinsic config expects a
  different axis convention (e.g. front-left-up vs. the Velodyne's own
  frame), you'll need to add a static transform or adjust the extrinsics,
  same as with any IMU/lidar pairing.
- **`playback_rate`** lets you slow down (e.g. `0.5`) if FAST-LIO can't
  keep up in real time, or speed up for faster offline testing.
- If lidar/oxts file counts don't match their timestamp files, the node
  logs a warning and just uses the shorter of the two — check that
  warning if your dataset directory looks incomplete.
