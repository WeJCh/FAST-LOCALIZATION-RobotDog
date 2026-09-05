# FAST-LOCALIZATION RobotDog 定位工程

本仓库是当前机器狗的在线定位工程。它使用 FAST-LIVO2 建图仓库产出的**三维关键帧先验地图**，在新的激光与 IMU 数据上完成全局初始化和持续定位；它不负责重新建图。

这是一个独立 Git 仓库。运行前需要准备另一个 `FAST-LIVO2 RobotDog` 建图仓库生成的一次完整地图运行目录，并保持两套工程的机器狗传感器配置一致。

## 1. 上游项目资料与适配说明

FAST-LOCALIZATION 是基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的已知地图重定位工程，使用 [Scan Context](https://github.com/gisbi-kim/scancontext_tro) 进行全局初始位姿检索，并使用连续的全局点云约束维持定位。与需要人工输入初始位姿的方案相比，上游设计的目标是自动全局初始化。

<p align="center">
  <img src="doc/test.gif" height="400" alt="上游室内自动初始化演示" />
  <br />
  <em>上游室内自动初始化演示。</em>
</p>

<p align="center">
  <img src="doc/init.gif" height="200" alt="上游 Scan Context 初始化演示" />
  <img src="doc/detail.gif" height="200" alt="上游定位细节演示" />
  <br />
  <em>上游 Scan Context 初始化与定位细节演示。</em>
</p>

当前仓库将输入改为机器狗的标准 `PointCloud2` 与 `Imu` 话题，并新增 `fast_livo2` 地图读取格式。上游文档中 `localization_mid360.launch`、Livox 话题和 HBA 地图目录结构属于另一套输入约定，不应直接套用到当前机器狗实验。

## 2. 定位流程

```text
前置激光雷达 + 雷达 IMU
          │
          ▼
临时激光惯性里程计
          │
          ├── Scan Context 候选检索
          ├── 粗/细 ICP 配准
          └── 连续匹配确认（至少两次一致匹配）
          │
          ▼
在 FAST-LIVO2 三维关键帧地图中的全局初始化
          │
          ▼
固定全局地图上的 IEKF 激光惯性连续定位
          │
          ├── /Odometry、/Path、TF
          ├── 配准后的点云
          └── map 坐标系轨迹 CSV 与结果点云
```

在“全局初始化确认”前，节点只在积累并验证候选匹配；此阶段不应把临时里程计当成最终全局位姿。控制台出现全局定位确认信息后，才开始输出可用于评估的 map 坐标系轨迹。

## 3. 机器狗输入与配置

当前适配配置为 [config/robotdog.yaml](config/robotdog.yaml)：


| 输入     | 默认话题           | 说明                                           |
| -------- | ------------------ | ---------------------------------------------- |
| 激光雷达 | `/front_lidar`     | `sensor_msgs/PointCloud2`，当前为 4 线雷达配置 |
| 雷达 IMU | `/front_lidar/imu` | `sensor_msgs/Imu`                              |
| 先验地图 | `map_dir` 参数     | FAST-LIVO2 的一次完整三维地图运行目录          |

当前定位启动器为 [launch/localization_robotdog.launch](launch/localization_robotdog.launch)。它默认使用 `map_format:=fast_livo2`，并读取雷达—IMU 外参与点云时间戳设置。

必须让本仓库和建图仓库中的 `robotdog.yaml` 保持相同的雷达—IMU 外参、雷达类型、扫描线数和时间戳单位。若机器狗更换了传感器安装位置或驱动消息字段，应先重新标定/核对这两份配置，再重建地图；仅修改定位侧参数不能修复地图坐标系不一致的问题。

## 4. FAST-LIVO2 先验地图要求

本工程不会读取二维导航栅格或单独的最终稠密点云作为 `fast_livo2` 地图。`$MAP_DIR` 必须指向 FAST-LIVO2 的一次建图运行目录，至少包含：

```text
<map_id>/
├── keyframes/
│   ├── metadata.yaml
│   └── 关键帧局部激光点云（零填充编号 .pcd）
└── loop_backend/
    └── optimized_keyframe_poses_imu.txt
```

加载时，程序用优化后的 IMU 位姿和雷达到 IMU 外参恢复每个关键帧在地图坐标系下的点云。若没有优化位姿文件，工程会尝试回退到关键帧位姿文件，但这不是回环优化后的交付地图，结果需要单独说明。

## 5. 部署：依赖安装、源码获取与编译

### 5.1 系统与基础库

上游 README 声明 Ubuntu >= 16.04、ROS >= Melodic；当前机器狗适配实际以 **Ubuntu 20.04 + ROS Noetic + catkin** 为目标。基础依赖为：

- PCL >= 1.8；
- Eigen >= 3.3.4；
- OpenCV >= 3.2；
- ROS 的 `pcl_ros`、`tf`、`eigen_conversions`、消息生成等组件。

若使用自定义 PCL，需让 CMake 能够找到其安装目录，并确认它与 ROS/PCL 相关依赖的 ABI 兼容。仅在 `~/.bashrc` 设置环境变量而未检查 CMake 配置输出，不能保证会使用预期版本。

### 5.2 获取源码与 ikd-Tree

当前仓库已经将 `include/ikd-Tree` 的源码直接纳入 Git 版本控制，因此正常克隆本仓库即可获得它。将以下地址替换为你发布到 GitHub 的定位仓库地址：

```bash
cd "$WORKSPACE/src"
git clone https://github.com/WeJCh/FAST-LOCALIZATION-RobotDog FAST-LOCALIZATION
```

原始上游 README 要求执行 `git submodule update --init`；这是因为其发布方式曾使用 ikd-Tree 子模块。当前仓库的 [.gitmodules](.gitmodules) 仍保留该上游记录，但 Git 索引中并没有子模块链接，执行该命令不会补充额外文件。若未来维护者重新将 ikd-Tree 改为子模块，应在 README 和克隆命令中同步恢复该初始化步骤。

### 5.3 Livox 依赖的适用范围

上游工程使用 `livox_ros_driver`。当前机器狗配置使用标准 `sensor_msgs/PointCloud2`，因此默认以 `FAST_LOCALIZATION_WITH_LIVOX=OFF` 编译，不需要 Livox 驱动。只有确实要接入 Livox `CustomMsg` 数据时，才安装 `livox_ros_driver` 并显式使用 `-DFAST_LOCALIZATION_WITH_LIVOX=ON` 重新配置。

### 5.4 编译

```bash
source /opt/ros/noetic/setup.bash
cd "$WORKSPACE"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DFAST_LOCALIZATION_WITH_LIVOX=OFF -j4
source devel/setup.bash
```

`$WORKSPACE` 是 catkin 工作空间根目录。若工作空间中同时保留建图与定位包，确认两个包均已被 catkin 发现后再运行；不要把两套同名可执行文件或旧版配置混入同一个启动命令。

## 6. 运行一次定位实验

以下命令假定 FAST-LIVO2 建图仓库位于同一工作空间的 `src/FAST-LIVO2`，并且已准备好一份独立的地图目录。先启动定位节点，再回放只含激光和 IMU 的机器狗数据。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
unset OMP_NUM_THREADS
roslaunch fast_localization localization_robotdog.launch \
  rviz:=false \
  map_format:=fast_livo2 \
  map_dir:="$MAP_DIR" \
  trajectory_dir:="$RESULT_DIR"
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
python3 "$WORKSPACE/src/FAST-LIVO2/scripts/play_ros2_robotdog_to_ros1.py" \
  "$DATASET_DIR" --rate 1.0 --skip-images --wait-subscribers
```

变量含义：

- `$MAP_DIR`：FAST-LIVO2 的 `<map_id>` 运行目录，而不是 `nav_map_terrain_final` 等二维地图目录；
- `$DATASET_DIR`：待定位的 rosbag2 数据目录；
- `$RESULT_DIR`：本次实验的空输出目录，建议每次使用新的目录名。

一次定位实验只能启动本仓库的定位节点；不要与 FAST-LIVO2 建图节点同时消费同一组激光和 IMU 话题。回放结束后等待节点写完轨迹与点云，再停止进程。

## 7. 输出、评估与结果解释

启用轨迹自动保存后，程序会在全局初始化确认后写出 map 坐标系的轨迹 CSV，并保存本次定位的点云结果。轨迹状态的平移量对应 IMU 位置；若外部真值测量的是天线或机体其它参考点，比较前需要使用已验证的刚体外参进行转换。

RTK/GNSS 数据目前仅作为离线对照，不参与在线初始化、Scan Context 检索、ICP 或滤波更新。没有独立确认以下条件前，不能把误差统计称为绝对定位精度或 ATE：

- RTK 坐标系与 FAST-LIVO2 地图坐标系的对应关系；
- RTK 天线、IMU 与机体参考点之间的杆臂；
- 激光、IMU、RTK 的时间基准和时间偏移；
- 全局初始化后有效轨迹与真值的时间段对齐规则。

建议将每次实验的地图版本、`robotdog.yaml` 版本、数据集标识、启动参数、全局初始化时刻和评估脚本版本一并写入可复现实验说明。大体积地图、rosbag、运行日志和结果点云应作为附件或发布资产管理，不建议直接提交到源码仓库。

更详细的输入输出说明见 [定位工作逻辑与输入输出说明.md](定位工作逻辑与输入输出说明.md)。

## 8. 致谢、上游来源与许可证

感谢 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)、[HBA](https://github.com/hku-mars/HBA) 和 [ScanContext](https://github.com/gisbi-kim/scancontext_tro) 的开源工作。本工程基于 FAST-LOCALIZATION 二次适配，保留上游版权、许可证和依赖声明。公开发布前，请同时核对 [LICENSE](LICENSE)、上游许可证、地图数据授权以及采集环境中可能涉及的隐私要求。
