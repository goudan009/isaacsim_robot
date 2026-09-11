# 仓库架构与边界

## 当前范围

`src/isaacsim_robot` 是唯一 Git 仓库边界，按“仿真核心、ROS 2 接口、工具、测试、报告”
分层。阶段 1-3 已归档到这里；阶段 4 的控制与传感器组合入口尚未设计，也没有把旧的
全身启动仓库照搬进来。

## 目录职责

```text
isaacsim_robot/
├── isaac_sim_core/
│   ├── assets/robots/         # canonical 机器人 USD
│   ├── assets/environments/  # 机器人、传感器独立和控制场景
│   ├── components/sensors/   # Isaac Sim 内的传感器消费逻辑
│   └── config/sensor_params/ # 传感器和跨层配置
├── ros2_pkgs/
│   ├── control/description/   # xacro -> Isaac URDF 生成器
│   ├── control/contract/      # 动作、观测、话题和频率合同
│   ├── control/bringup/       # 控制辅助 launch、脚本和 controller YAML
│   └── simulation_bridge/sensor_pkg/ # RealSense/MID360 ROS 2 适配
├── tools/                    # 资产导出、边界校验、模型工具
├── scripts/                  # 仓库级运维脚本
├── test/                     # 静态、单元和 ROS 2 集成测试
├── reports/                  # 迁移和性能结论
└── docs/                     # 本文档和运行性能指南
```

## ROS 2 包

| 包 | 责任 | 当前不负责 |
| --- | --- | --- |
| `isaacsim_description` | 生成 Isaac Sim 专用 URDF | 不保存生成物，不启动整机 |
| `isaacsim_embodiment_contract` | 固定动作、观测、关节顺序、topic 和频率合同 | 不实现任务策略或传感器渲染 |
| `isaacsim_bringup` | 控制器参数、RViz/VR 辅助入口、spawn 和运行验证脚本 | 不提供阶段 4 的全身组合入口 |
| `isaacsim_sensors` | RealSense/MID360 rig、ROS 2 发布适配、队列和诊断 | 不拥有机器人 USD 或控制器 |

`ros2_pkgs/` 下的目录和 ROS 2 包名不再使用旧的多仓库前缀。代码中保留的机器人话题、
frame 和上游依赖名称属于运行接口兼容项，不代表重新拆分仓库。

## 数据流与依赖方向

```text
机器人 USD ──> Isaac Sim 场景 ──> ROS 2 control/controller manager
                                  ├── /cmd_vel、关节 command
                                  └── /joint_states、/odom

独立传感器配置 ──> RealSense rig / MID360 RTX LiDAR
                         ├── 图像、depth、CameraInfo、TF
                         ├── PointCloud2、IMU
                         └── 有界队列和诊断
```

控制层不依赖传感器包；传感器层不依赖控制器管理器。阶段 4 如果重新组合，必须通过
明确的配置和组合层连接，不能把传感器实现塞进全身启动脚本，也不能把控制回调用于
渲染、序列化或写盘。

## Canonical 文件

| 内容 | 唯一维护位置 |
| --- | --- |
| 机器人 USD | `isaac_sim_core/assets/robots/openflex_robot.usda` |
| 机器人独立场景 | `isaac_sim_core/assets/environments/robot_only_stage.usda` |
| 控制场景 | `isaac_sim_core/assets/environments/robot_control_stage.usda` |
| 控制器 | `ros2_pkgs/control/bringup/config/controllers.isaac.mobile_base.yaml` |
| 动作/观测合同 | `ros2_pkgs/control/contract/config/embodiment.yaml` |
| RealSense 独立/机器人挂载配置 | `isaac_sim_core/config/sensor_params/realsense/` |
| MID360 挂载/语义配置 | `isaac_sim_core/config/sensor_params/mid360/` |
| 传感器目标 topic | `isaac_sim_core/config/sensor_params/sensors.isaac.yaml` |

ROS 2 安装态的 `ros2_pkgs/simulation_bridge/sensor_pkg/config/` 是包安装需要的配置副本；
修改配置时必须同步 canonical 配置，并通过测试确认两者没有漂移。

## 阶段边界

- 阶段 1：机器人资产、挂点和机器人场景，已完成。
- 阶段 2：RealSense/MID360 独立资产、配置和传感器适配，已完成基础功能验证。
- 阶段 3：机器人描述、ROS 2 控制配置、接口合同和验证脚本，已完成静态/隔离构建验证。
- 阶段 4：控制与传感器产品化组合、严格状态快照 IPC、整机实时验收，已完成性能候选方案验证，
  但正式产品化入口仍未验收。已进行只读状态快照与相机副本的双进程架构实验；该实验不等于
  正式产品化实现，结论见
  [`../reports/sensors/sensor_validation.md`](../reports/sensors/sensor_validation.md)。
- 阶段 5：整机组合测试验收，已满足入口性能线，当前进入验收阶段。验收重点是正式启动链路、
  动态雷达外参与时间一致性、重复运行稳定性和故障退出处理。

在阶段 5 验收完成前，不创建 `all_body` 类全身入口，不加入 SLAM/语义建图组合启动，
也不把已有独立传感器测试改写成整机验收结论。
