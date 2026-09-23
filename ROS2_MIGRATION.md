# FAST-LOCALIZATION ROS 2 Humble migration

This worktree is the ROS 2 migration target. The sibling `FAST-LOCALIZATION`
worktree remains the unmodified ROS 1/Noetic comparison baseline.

## Scope of this first ROS 2 version

- Builds as an `ament_cmake` package named `fast_localization`.
- Consumes standard `sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Imu`.
- Preserves the RobotDog input contract: `/front_lidar` fields must be
  `x`, `y`, `z`, `intensity` (`float32`), `tag`, `line` (`uint8`), and a
  `timestamp` (`float64`) that is a per-scan nanosecond offset.
- Uses Best Effort / Volatile QoS with depth 10 for LiDAR and depth 100 for
  IMU, matching the recorded RobotDog interface.
- Loads existing FAST-LIVO2 maps unchanged, including
  `keyframes/metadata.yaml` and optimized keyframe poses.
- Keeps the original topic names for regression comparison. TF publication is
  disabled by default and must be enabled explicitly with `publish_tf:=true`.

The old ROS 1 Livox `CustomMsg` input is intentionally not included. The ROS 1
baseline already built it only when an optional flag was enabled, while the
RobotDog target publishes standard PointCloud2. A future AVIA port needs an
explicit `livox_ros_driver2` adapter and its message-version validation.

## Build on the Humble test server

Place this directory at `~/ros2_ws/src/fast_localization` (the directory name
is arbitrary; the package name is not), then run:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_localization --symlink-install
source install/setup.bash
```

The build host needs Ubuntu 22.04, ROS 2 Humble, PCL, OpenCV, Eigen, OpenMP,
and the ROS packages declared in `package.xml`. `rosdep` is the authoritative
dependency resolver; do not copy Noetic `catkin` dependencies into this
workspace.

## Offline RobotDog bag regression

In one terminal, with the workspace sourced:

```bash
ros2 launch fast_localization localization_robotdog.launch.py \
  map_dir:=/absolute/path/to/FAST-LIVO2-run \
  trajectory_dir:=/tmp/fast_localization_trajectory \
  use_sim_time:=true
```

In another terminal:

```bash
ros2 bag play /absolute/path/to/robotdog_bag --clock
```

The expected input topics are `/front_lidar` and `/front_lidar/imu`. Check
that `/Odometry`, `/path`, `/cloud_registered`, and `/global_map` are present;
the node logs `Global localization confirmed` only after its configured number
of consistent ScanContext/ICP candidates.

## Rollback

Every migration change belongs to the `ros2-humble` branch only. To discard a
bad local change, reset or check out the previous `ros2-humble` commit in this
worktree. The ROS 1 baseline directory and its branch are never edited by this
workflow.
