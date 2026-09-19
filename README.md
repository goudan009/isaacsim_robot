# OpenFleX Isaac Sim Robot

OpenFleX 的独立 Isaac Sim 6.0 + ROS 2 Humble 仿真仓库。机器人仿真资产、六个 ROS 2 包、
接口合同、测试和运行报告都在本仓库维护，不再把仿真源码拼接到 `openflex_ws/src`。

## 仓库结构

```text
isaacsim_robot/
├── isaac_sim_core/                  # USD、场景、机器人和传感器 canonical 配置
├── ros2_pkgs/openflex_isaac_sim/    # 六个 ROS 2 包
├── config/dependencies.repos        # 固定版本的第三方源码依赖
├── docs/                            # 架构、运行和性能边界
├── reports/                         # 可审阅的迁移、性能和运行证据
├── test/                            # 静态、ROS 2 集成和性能测试
└── tools/                           # 资产和模型工具
```

ROS 2 包包括 `openflex_isaac_description`、`openflex_isaac_contract`、
`openflex_isaac_controllers`、`openflex_isaac_sensors`、`openflex_isaac_bridge` 和
`openflex_isaac_bringup`。旧 `ros2_pkgs/control`、`ros2_pkgs/simulation_bridge` 和
`isaacsim_*` 包名已经移除。

## 依赖边界

本仓库是独立 Git 仓库，但运行时仍需要 ROS 2 underlay 提供 OpenFleX/硬件侧依赖，主要包括：

- `livox_ros_driver2`
- `swerve_controller`
- `isaac_ros2_scripts`
- `topic_based_ros2_control`

`isaac_ros2_utils` 已固定在 `config/dependencies.repos` 中。不要把它的嵌套 `.git` 目录提交到
本仓库。OpenFleX 主工作空间可以作为 underlay，但仿真源码只维护在本仓库。

## 获取依赖

```bash
cd /home/1024201092WYH/Robot/isaacsim_robot
mkdir -p .deps/src
vcs import .deps/src < config/dependencies.repos
```

如果 OpenFleX underlay 已经安装上述依赖，可以直接 source underlay，不必重复构建同名包。

## 构建

```bash
cd /home/1024201092WYH/Robot/isaacsim_robot
source /opt/ros/humble/setup.bash
source /home/1024201092WYH/Robot/openflex_ws/install/setup.bash

colcon build --symlink-install \
  --base-paths ros2_pkgs/openflex_isaac_sim \
  --allow-overriding \
    openflex_isaac_bridge \
    openflex_isaac_bringup \
    openflex_isaac_contract \
    openflex_isaac_controllers \
    openflex_isaac_description \
    openflex_isaac_sensors

source install/setup.bash
```

独立仓库必须最后 source，确保六个 `openflex_isaac_*` 包解析到本仓库的 `install/`。

## 启动

```bash
cd /home/1024201092WYH/Robot/isaacsim_robot
source /opt/ros/humble/setup.bash
source /home/1024201092WYH/Robot/openflex_ws/install/setup.bash
source install/setup.bash

export ISAACSIM_ROBOT_ROOT="$PWD"
export ROS_DOMAIN_ID=49
export ROS_LOCALHOST_ONLY=1

ros2 launch openflex_isaac_bringup sim.launch.py \
  headless:=true \
  render_hz:=30 \
  physics_hz:=120 \
  sensor_profile:=full \
  lidar_transport:=helper \
  lidar_mount_mode:=parented \
  lidar_object_id_map:=false \
  livox_max_points:=15000 \
  start_upper_body:=true \
  isaac_path:=/home/1024201092WYH/isaacsim-6.0 \
  api_port:=8085 \
  ros_domain_id:=49
```

主 launch 不启动 RViz。第二个终端使用相同的 ROS 环境后运行：

```bash
ros2 launch openflex_isaac_bringup rviz_only.launch.py \
  use_sim_time:=true ros_domain_id:=49
```

## 默认接口

控制接口：

- `/cmd_vel`
- `/left_forward_position_controller/commands`
- `/right_forward_position_controller/commands`
- `/head_forward_position_controller/commands`
- `/lift_position_controller/commands`
- `/velocity_controller/commands`

观测接口：

- `/joint_states`
- `/odom`
- `/fastlio2/lio_odom`
- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`
- `/livox/lidar_points`：`sensor_msgs/msg/PointCloud2`
- `/openflex/livox_frame/lidar`：Isaac 原始 `PointCloud2`
- `/scan`
- `/livox/imu`
- `/cam_{base,head,left,right}/{color,depth}/image`
- `/cam_{base,head,left,right}/color/image/compressed`

RViz 使用 `/livox/lidar_points`。需要 Livox CustomMsg 的 FAST-LIO 使用 `/livox/lidar`。

## FAST-LIO 与 VLA 边界

- `/livox/lidar` 的消息类型、逐点时间和 frame 已按真机 Livox 输入合同发布，可以接入要求
  `livox_ros_driver2/msg/CustomMsg` 的 FAST-LIO。
- `/fastlio2/lio_odom` 当前只是仿真 `/odom` 的兼容转发，不是 FAST-LIO 算法输出。
- 仓库不自动启动 FAST-LIO，也不替代 FAST-LIO 参数、外参、时间同步和轨迹 ATE/RPE 验收。
- 仓库提供 VLA 所需的观测和动作接口，但不包含数据集落盘、训练任务、checkpoint 和策略推理代码。
- 因此当前可以接入上层 VLA 数据采集器，但不能仅凭本仓库宣称完整“采集-训练-推理”闭环已完成。

## 验证

```bash
python3 -m pytest -q test

source install/setup.bash
ros2 run openflex_isaac_contract verify_embodiment_contract.py
ros2 control list_controllers
ros2 run openflex_isaac_bringup verify_vla_runtime.py \
  --sample-seconds 15 \
  --min-lidar-points 10000 \
  --min-lidar-hz 5
ros2 run openflex_isaac_bringup verify_camera_images.py
```

2026-09-19 的独立仓库实测中：六个控制器为 `active`；四路 VLA JPEG 约 15 Hz；
`/livox/lidar` 在每帧 15,000 点时约 6.55 Hz；`/livox/lidar_points` 中位约 69,787 点、
约 6.37 Hz。`nearRangeM=0.1` 修复了启动后长期只有约 500 点/帧的问题。

报告保存在 `reports/runtime/full_chain/`。构建目录、生成 URDF、原始日志和第三方嵌套仓库不提交。

## 详细文档

- `docs/ARCHITECTURE_CN.md`
- `docs/RUNTIME_PERFORMANCE_CN.md`
- `docs/REPORTING_CN.md`
- `ros2_pkgs/openflex_isaac_sim/openflex_isaac_bringup/README.md`
