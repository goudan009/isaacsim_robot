# MRS_ROBOT_sim

OpenFleX 的独立 Isaac Sim 6.0 + ROS 2 Humble 仿真仓库。机器人仿真资产、六个 ROS 2 包、
接口合同、测试和运行报告都在本仓库维护，不再把仿真源码拼接到 `openflex_ws/src`。

## 与 OpenFleX 控制中心协作部署

`MRS_ROBOT_sim` 是独立维护、独立构建的 Isaac Sim + ROS 2 仿真仓库；GUI 只通过 ROS 2 launch 和 topic/消息合同调用它。
推荐在集成目录中与 `openflex_ws` 并列放置，**不要放进 `openflex_ws/src`**：

```text
openflex_all/                 # 集成目录；当前阶段不把仿真仓库提交进主仓库
├── openflex_ws/              # GUI、真机 ROS 包及仿真所需 underlay
├── MRS_ROBOT_sim/              # Isaac Sim + ROS 仿真运行时，独立提交和发布
└── MRS_ROBOT_arena/            # Arena 原生任务和策略评测，独立提交和发布
```

当前阶段不使用主仓库分支或 submodule。部署成员应从团队确认的 Isaac 仿真仓库地址单独克隆；
团队仓库地址为 `git@github.com:mrs-lab-robot/MRS_ROBOT_sim.git`。本地 `origin` 应指向该团队仓库；
原 `goudan009/isaacsim_robot` 地址仅作为迁移来源保留，不再用于新的部署。集成目录的主仓库
不要执行 `git add .` 来收录这个嵌套仓库。若未来需要让主仓库锁定已验收的仿真版本，再由双方维护者
评估 Git submodule；届时主仓库只记录仿真仓库 URL 和提交 SHA，不复制仿真源码或历史。

```bash
export OPENFLEX_ALL_ROOT=/绝对路径/openflex_all
export OPENFLEX_WS_ROOT="$OPENFLEX_ALL_ROOT/openflex_ws"
export ISAACSIM_ROBOT_ROOT="$OPENFLEX_ALL_ROOT/MRS_ROBOT_sim"  # 兼容保留的变量名
export ISAACSIM_PATH=/绝对路径/isaacsim-6.0
export ISAACSIM_GIT_URL=git@github.com:mrs-lab-robot/MRS_ROBOT_sim.git

git clone "$ISAACSIM_GIT_URL" "$ISAACSIM_ROBOT_ROOT"
```

上面的路径只是示例，按每台机器的实际目录修改。**GUI 所填的软件路径属于所选运行目标**：
选择本机时填本机路径；选择 SSH 工作站时，仿真仓库、OpenFleX 工作区、ROS 和 Isaac Sim
必须已部署在远端，并填写远端路径。本机克隆不会自动出现在 4090/6000 Ada 工作站上。

GUI 启动合同目前要求：

- `ISAACSIM_ROBOT_ROOT/ros2_pkgs/openflex_isaac_sim/openflex_isaac_bringup/launch/sim.launch.py` 存在。
- 仿真工作区已构建，且 `ISAACSIM_ROBOT_ROOT/install/setup.bash` 存在。
- source 仿真工作区后，`ros2 pkg prefix openflex_isaac_bringup` 指向该仓库的 `install/`。
- `ISAACSIM_PATH/python.sh` 存在且可执行。
- ROS 环境按 ROS 2 Humble → OpenFleX underlay → 本仓库 overlay 的顺序加载。

### 空目录的版本控制

`openflex_isaac_description/CMakeLists.txt` 会安装 `meshes/` 和 `launch/`。这两个目录目前允许为空，
但 Git 不保存空目录；6000 Ada 上的旧克隆中它们存在，新克隆中却消失，导致干净构建失败。目录内的
`.gitkeep` 是有意保留的占位文件，保证 clone 后构建行为一致。今后若加入网格或 launch 文件，直接将
真实资源放入对应目录并保留目录，不要删除 CMake 安装规则；也不要依赖部署机上手工创建的未跟踪目录。

修改 launch 文件名、launch 参数、包名、ROS Domain/RMW 设置或控制/传感器 topic 时，仿真维护者
需要同步更新本仓库文档与合同测试，并通知 GUI 维护者检查
`openflex_gui/isaacsim_workflow.py` 的预检和启动参数。**不要为了让 GUI 通过预检而改写或复制仿真源码**；
先确认两边的接口约定确实发生了变化。

### 团队维护与验收顺序

1. 在目标机安装匹配版本的 Isaac Sim、ROS 2 Humble 和 OpenFleX underlay；安装目录不属于本仓库，
   不要将大型 Isaac Sim 安装包提交到 Git。
2. 从团队仓库克隆本仓库到 `openflex_all/MRS_ROBOT_sim`，按下方“获取依赖”和“构建”步骤单独构建，
   再 source 本仓库的 `install/setup.bash`。
3. 在 GUI 仿真页按目标机分别填写路径，先运行“检查仿真环境”。检查通过只证明 ROS/包/文件路径满足
   启动合同，不代表 Isaac Sim 已经实际运行。
4. 首次启动建议使用无界面和 `none` 传感器档位做轻量冒烟测试；确认控制器、`/joint_states`、`/odom`
   后再启用 RGB-D/MID360。停止仿真后，按项目测试和运行验收命令检查消息频率与传感器内容。
5. 仿真仓库的提交、review、tag/release 均在本仓库独立完成。发布前运行本 README 的验证命令；
   涉及 ROS 接口时同时更新 `docs/ARCHITECTURE_CN.md`、相关 package README 和合同测试。

构建生成的 `build/`、`install/`、`log/`、`.deps/`、第三方源码和原始运行日志属于机器本地内容，
不得提交。团队共享的测试结论应整理为可审阅的报告，而不是把完整临时日志或 Isaac Sim 安装目录入库。

## 仓库结构

```text
MRS_ROBOT_sim/
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
export OPENFLEX_ALL_ROOT=/绝对路径/openflex_all
export OPENFLEX_WS_ROOT="$OPENFLEX_ALL_ROOT/openflex_ws"
export ISAACSIM_ROBOT_ROOT="$OPENFLEX_ALL_ROOT/MRS_ROBOT_sim"
cd "$ISAACSIM_ROBOT_ROOT"
mkdir -p .deps/src
vcs import .deps/src < config/dependencies.repos
```

如果 OpenFleX underlay 已经安装上述依赖，可以直接 source underlay，不必重复构建同名包。

## 构建

```bash
export OPENFLEX_ALL_ROOT=/绝对路径/openflex_all
export OPENFLEX_WS_ROOT="$OPENFLEX_ALL_ROOT/openflex_ws"
export ISAACSIM_ROBOT_ROOT="$OPENFLEX_ALL_ROOT/MRS_ROBOT_sim"
cd "$ISAACSIM_ROBOT_ROOT"
source /opt/ros/humble/setup.bash
source "$OPENFLEX_WS_ROOT/install/setup.bash"

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
export OPENFLEX_ALL_ROOT=/绝对路径/openflex_all
export OPENFLEX_WS_ROOT="$OPENFLEX_ALL_ROOT/openflex_ws"
export ISAACSIM_ROBOT_ROOT="$OPENFLEX_ALL_ROOT/MRS_ROBOT_sim"
export ISAACSIM_PATH=/绝对路径/isaacsim-6.0
cd "$ISAACSIM_ROBOT_ROOT"
source /opt/ros/humble/setup.bash
source "$OPENFLEX_WS_ROOT/install/setup.bash"
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
  isaac_path:="$ISAACSIM_PATH" \
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
