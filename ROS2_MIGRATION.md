# FAST-LOCALIZATION ROS 2 Humble 迁移说明

本目录是 ROS 2 的唯一迁移目标；同级的 `FAST-LOCALIZATION` 工作目录仍是
未修改的 ROS 1/Noetic 对照基线。

## 当前状态

ROS 2 源码迁移已完成并提交到 `ros2-humble` 分支，但**尚未在 Ubuntu 22.04 +
ROS 2 Humble 上完成编译和 rosbag 回放验证**。因此目前的准确状态是“可供
Humble 服务器验证的迁移版本”，而不是“已经验证可运行的版本”。

## 本次 ROS 2 版本的范围

- 使用 `ament_cmake` 构建，包名为 `fast_localization`；
- 订阅标准的 `sensor_msgs/msg/PointCloud2` 与 `sensor_msgs/msg/Imu`；
- 保持机器狗输入约定：`/front_lidar` 必须具有 `x`、`y`、`z`、`intensity`
  （`float32`）、`tag`、`line`（`uint8`）以及 `timestamp`（`float64`）字段；
  `timestamp` 必须是扫描内相对起点的纳秒偏移；
- LiDAR 使用 Best Effort / Volatile / depth 10，IMU 使用 Best Effort /
  Volatile / depth 100，与已检查的机器狗录包接口一致；
- 原样读取 FAST-LIVO2 地图，包括 `keyframes/metadata.yaml` 与优化后的关键帧位姿；
- 为便于与 ROS1 回归对比，保持原有输出话题名；TF 默认关闭，只有显式传入
  `publish_tf:=true` 才会发布 `camera_init -> body`。

旧版 ROS1 的 Livox `CustomMsg` / AVIA 输入没有迁移。这是有意的范围限制：
机器狗目标使用标准 PointCloud2，且 ROS1 基线原本也只有在启用可选编译开关后
才支持 `CustomMsg`。若后续需要 AVIA，应单独接入 `livox_ros_driver2`，并验证其
消息定义与时间戳语义。

## 在 Humble 测试服务器上构建

将本目录同步到 `~/ros2_ws/src/fast_localization`（目录名可以不同，包名必须是
`fast_localization`），然后执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_localization --symlink-install
source install/setup.bash
```

服务器需要 Ubuntu 22.04、ROS 2 Humble、PCL、OpenCV、Eigen、OpenMP 以及
`package.xml` 中声明的 ROS 依赖。应以 `rosdep` 的解析结果为准，不应把 Noetic
的 catkin 依赖直接复制到此工作空间。

## 使用机器狗 ROS2 bag 做离线回归

终端一（已 source 工作空间）：

```bash
ros2 launch fast_localization localization_robotdog.launch.py \
  map_dir:=/absolute/path/to/FAST-LIVO2-run \
  trajectory_dir:=/tmp/fast_localization_trajectory \
  use_sim_time:=true
```

终端二：

```bash
ros2 bag play /absolute/path/to/robotdog_bag --clock
```

预期输入话题为 `/front_lidar` 与 `/front_lidar/imu`。至少检查 `/Odometry`、
`/path`、`/cloud_registered`、`/global_map` 是否持续发布；只有 ScanContext 与
ICP 连续候选满足配置条件后，日志才会出现 `Global localization confirmed`。

## 回退

迁移改动只存在于 `ros2-humble` 分支。若本机后续修改失败，可在这个工作目录
切换或回退至上一个已验证提交；ROS1 基线目录及其 `main` 分支不受影响。
