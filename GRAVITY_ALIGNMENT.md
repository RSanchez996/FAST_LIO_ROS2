# Gravity-aligned mapping

This fork can initialize FAST-LIO in a world frame whose `+Z` axis is opposite
to gravity. The robot may start with non-zero roll and pitch, but it must remain
stationary until initialization completes.

## Geometry

At rest, a REP-145 IMU measures specific force approximately opposite to
gravity. The initializer estimates the mean vector `a_imu` and constructs the
minimum rotation `R_world_imu` satisfying:

```text
R_world_imu * normalize(a_imu) = [0, 0, 1]
gravity_world                 = [0, 0, -g]
```

Gravity does not observe yaw. REP-105 startup alignment therefore defaults to
`yaw_only`, which can align yaw and XY with `odom` without reintroducing roll or
pitch into the global map.

## Startup requirements

1. Place the robot still. It does not need to be mechanically level.
2. Start the LiDAR, IMU and FAST-LIO stack.
3. Do not move the robot while the node reports
   `Waiting for stationary IMU initialization`.
4. Begin mapping only after `Gravity reference finalized` appears.

The stationary candidate is reset when angular velocity or acceleration norm
exceeds its configured limit. A candidate is accepted only if both its duration
and sample count are sufficient and the per-axis accelerometer/gyroscope
standard deviations pass.

## Recommended MID-360 parameters

```yaml
gravity_alignment:
  enabled: true
  require_stationary: true
  min_duration_sec: 3.0
  min_samples: 400
  max_samples: 4000
  max_gyro_norm_rad_s: 0.08
  accel_norm_tolerance_m_s2: 0.60
  max_accel_std_m_s2: 0.25
  max_gyro_std_rad_s: 0.02
  gravity_magnitude_m_s2: 9.80665
  # livox_ros_driver2 publishes MID-360 acceleration in g.
  input_accel_scale_to_m_s2: 9.80665

rep105:
  align_map_to_odom_on_start: true
  initial_alignment_mode: yaw_only
  project_map_to_2d: false
```

`input_accel_scale_to_m_s2` converts the acceleration carried by the incoming
`sensor_msgs/Imu` message before applying thresholds or feeding the filter.
Use `9.80665` with the MID-360 and `livox_ros_driver2` (input norm near `1 g`).
Use `1.0` for a REP-145/ROS-compliant IMU that already publishes m/s² (input
norm near `9.80665`). The startup log prints the converted acceleration norm so
that a unit mismatch is immediately visible.

`full_6d` is rejected while gravity alignment is enabled because it can rotate
the already-level FAST-LIO world back onto an inclined `odom` frame. `identity`
is also valid when no startup match to `odom` is required.

## Exported metadata

The compact map receives a sidecar named:

```text
<map>.metadata.yaml
```

Binary PLY output additionally receives header comments for `gravity`,
`gravity_magnitude_m_s2`, `gravity_aligned`, `gravity_source` and `frame_id`.
Temporal-export `metadata.yaml` contains the full initialization statistics and
the row-major `R_world_imu` matrix. These records allow downstream tools to
verify the frame rather than estimate another unrelated leveling rotation.

## Verification

After mapping, check a PLY header with:

```bash
sed -n '/^comment gravity/p;/^end_header/q' MAP.ply
```

For a correctly aligned output frame it should contain approximately:

```text
comment gravity 0 0 -1
comment gravity_aligned true
```

The downstream temporal cleaner and mesh generator must preserve this frame.
They must not independently rotate only one artifact, because RMCL and Nav2
must consume geometry expressed in exactly the same `map` coordinates.
