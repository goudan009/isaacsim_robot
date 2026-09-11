# OpenFleX Isaac Sim Robot

统一的 Isaac Sim 机器人仓库，位于 colcon 工作空间的 `src/isaacsim_robot`。
机器人资产、传感器资产、ROS 2 控制接口、工具、测试和报告在同一个 Git 边界内维护。

当前已完成阶段 1-3：

1. 机器人本体资产和机器人专用场景。
2. RealSense、MID360 资产、配置和独立传感器逻辑。
3. 机器人 ROS 2 描述、控制器配置、接口合同和验证脚本。

阶段 4“机器人 ROS 2 控制 + 传感器”的性能候选方案已完成验证，但正式产品化入口尚未验收。
阶段 5 整机组合测试已达到入口性能线，下一步进行启动链路、动态外参、时间一致性和重复稳定性
验收。雷达、SLAM 和旧的全身组合编排没有复制进本仓库。

## 无传感器控制启动

机器人控制入口为 `isaacsim_bringup robot_control_only.launch.py`。它只启动机器人 URDF、
Isaac Sim 控制图、`controller_manager`、`robot_state_publisher` 和可选 RViz，不启动相机、
雷达、IMU、传感器桥、SLAM 或兼容桥。

```bash
ros2 launch isaacsim_bringup robot_control_only.launch.py \
  headless:=true rviz:=false
```

默认启动无传感器完整控制链：底盘、双臂、头部和升降控制器均会加载。用于隔离底盘性能时，
显式传入 `start_upper_body:=false`。RViz 配置已包含底盘、双臂、头部和升降面板。

默认 Isaac Sim 物理频率为 120 Hz，`controller_manager`（包含公共 `/joint_states` 发布链）调度目标为 90 Hz。两者不是同一个
指标：前者是仿真步频，后者是 ROS 2 控制循环目标，最终性能必须以墙钟采样和控制响应验收。

无传感器控制入口默认 `render_hz:=30`：该频率可满足诊断画面采集，同时完整机器人实测仍超过
实时。需要更高刷新率的交互式 Isaac 视口时可显式传入 `render_hz:=60`，但该配置当前不满足
RTF `1.0` 的实时线。

当前实测记录见 [`reports/performance/robot_control_performance.md`](reports/performance/robot_control_performance.md)。
其中 `/joint_states` 必须看测试节点对该 ROS 话题的直接接收频率；physics step rate 必须看
由 `OnPhysicsStep` 触发的 `/openflex/joint_states` 仿真时间戳；RTF 必须由同一测量窗口的
仿真时间差和墙钟时间差计算。三者不能用配置目标或 `/clock` 消息频率相互替代。没有真机
ROS 进程时，也不能把真机 `/joint_states` 的 90 Hz 配置目标写成实测频率。

当前结果统一见 [`reports/README.md`](reports/README.md)：默认无传感器完整控制已通过底盘、升降、
头部和双臂的实际运动验证，实测 `/joint_states` 约 `90 Hz`、physics step rate `175.689 Hz`、
RTF `1.464`；空场景 RTF 约 `1.977`。历史重试只保留原始证据，不复制到结论正文。

## 目录

```text
isaacsim_robot/
├── isaac_sim_core/       # USD、仿真组件和传感器/物理配置
├── ros2_pkgs/             # colcon ROS 2 包
│   ├── control/           # description、contract、bringup
│   └── simulation_bridge/ # sensor_pkg
├── tools/                 # 资产导出、校验和模型工具
├── scripts/               # 仓库级脚本
├── test/                  # sim unit、ROS 2 integration、migration tests
├── reports/               # 精选迁移和性能结论
├── docs/                  # 架构、运行和性能说明
└── config/                # 工作空间级依赖清单
```

ROS 2 包名为 `isaacsim_bringup`、`isaacsim_command_controller`、`isaacsim_description`、
`isaacsim_embodiment_contract` 和 `isaacsim_sensors`；`ros2_pkgs` 下不使用旧仓库名称。

## 快速验证

```bash
source /opt/ros/humble/setup.bash
colcon list
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q test
```

需要 GPU/Isaac Sim 的运行验证时，使用 Isaac Sim 自带 Python，并先阅读
[`docs/RUNTIME_PERFORMANCE_CN.md`](docs/RUNTIME_PERFORMANCE_CN.md)。实时性能不能用静态
测试结果代替。

## 入口文档

- [`docs/README_CN.md`](docs/README_CN.md)：文档索引和当前状态。
- [`docs/ARCHITECTURE_CN.md`](docs/ARCHITECTURE_CN.md)：目录、边界和数据流。
- [`docs/RUNTIME_PERFORMANCE_CN.md`](docs/RUNTIME_PERFORMANCE_CN.md)：机器人控制、RealSense、
  MID360 的接入方式、性能保证方法和已知门槛。
- [`docs/REPORTING_CN.md`](docs/REPORTING_CN.md)：固定报告、批次格式、测评线和后续 agent
  的写入规则。
- [`reports/migration/stages_1_to_3.md`](reports/migration/stages_1_to_3.md)：迁移和构建测试结论。
- [`reports/sensors/sensor_validation.md`](reports/sensors/sensor_validation.md)：传感器功能、
  ROS 2 互通和性能证据。

## 维护规则

- `isaac_sim_core/` 保存仿真核心和 canonical 配置；`ros2_pkgs/` 只保存可由 colcon 识别的包。
- 测试代码放 `test/`，可审阅的结论放 `reports/`，原始日志和构建产物不提交。
- 修改传感器或控制配置后，同时更新对应报告中的测量条件和判定。
- 阶段 5 验收完成前，不实现未经验证的全身组合启动；继续使用三进程候选方案和固定报告记录结果。
