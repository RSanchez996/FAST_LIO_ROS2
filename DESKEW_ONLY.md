# Deskew-only mode

New parameters:

```yaml
processing:
  deskew_only: true

publish:
  deskewed_topic: /cloud_deskewed
```

Kept in this mode:

- Livox `CustomMsg` preprocessing with exact per-point `offset_time`.
- LiDAR/IMU buffering and synchronization.
- IMU initialization and ESKF prediction.
- Backward point motion compensation to the scan end.
- `sensor_msgs/msg/PointCloud2` publication in `frames.lio_body_frame`.

Skipped:

- Local map segmentation and ikd-tree construction.
- LiDAR scan-to-map correction.
- Map insertion.
- Odometry, path, map and TF publication.
- PCD map service.

Run:

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360_deskew_only.yaml \
  rviz:=false
```

Output:

```text
/cloud_deskewed [sensor_msgs/msg/PointCloud2]
```

The deskew trajectory is IMU-predicted and no longer receives LiDAR scan-to-map
corrections. This is a transitional low-cost mode. A production deskewer should
anchor pose and velocity to robot odometry or another external state estimate.
