# Deskew-only mode

This mode consumes the MID-360 native Livox message and publishes only a
motion-compensated ROS 2 point cloud:

```text
/livox/lidar     [livox_ros_driver2/msg/CustomMsg]
/livox/imu       [sensor_msgs/msg/Imu]
        |
        v
/cloud_deskewed  [sensor_msgs/msg/PointCloud2, frame_id=livox_frame]
```

The Livox driver must run with `xfer_format:=1`. Do not start a second driver
to obtain PointCloud2: all consumers that need PointCloud2 should subscribe to
`/cloud_deskewed`.

Run only FAST-LIO's deskewer after the Livox driver is active:

```bash
ros2 launch fast_lio deskew_only.launch.py
```

The launch forces the interface contract even if another compatible MID-360
parameter file is selected:

```yaml
processing.deskew_only: true
preprocess.lidar_type: 1
common.lid_topic: /livox/lidar
common.imu_topic: /livox/imu
publish.deskewed_topic: /cloud_deskewed
frames.lidar_frame: livox_frame
```

Kept in this mode:

- Livox `CustomMsg` preprocessing with exact per-point `offset_time`.
- LiDAR/IMU buffering and synchronization.
- IMU initialization and ESKF prediction.
- Backward point motion compensation to the scan end.
- `sensor_msgs/msg/PointCloud2` publication in the physical LiDAR frame.

Skipped:

- Local map segmentation and ikd-tree construction.
- LiDAR scan-to-map correction.
- Map insertion.
- Odometry, path, map and TF publication.
- PCD map service.

The deskew trajectory is IMU-predicted and no longer receives LiDAR scan-to-map
corrections. This is a low-cost mode. Translation compensation can drift during
long or highly dynamic motion because it is not anchored to robot odometry.
The Go2 driver remains the sole owner of `odom -> base_link`.
