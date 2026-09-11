# 阶段 1-3 迁移报告

日期：2026-09-06
范围：机器人本体资产、传感器资产、机器人 ROS 2 控制

## 结果

阶段 1-3 已迁移到同一个 `src/isaacsim_robot` Git 仓库，并完成布局、包识别、隔离构建
和静态/单元测试。阶段 4“机器人 ROS 2 控制 + 传感器”没有开始，旧的全身启动、SLAM
和组合编排没有复制。

| 阶段 | 归档内容 | 目标路径 | 状态 |
| --- | --- | --- | --- |
| 1 | 机器人 USD、机器人场景、资产导出/校验工具 | `isaac_sim_core/assets/`、`tools/model_converter/robot_assets/` | 已完成 |
| 2 | RealSense、MID360、独立 rig、配置和语义处理 | `isaac_sim_core/config/sensor_params/`、`isaac_sim_core/components/sensors/`、`ros2_pkgs/simulation_bridge/sensor_pkg/` | 已完成 |
| 3 | URDF 生成、控制器 YAML、接口合同、控制验证脚本 | `ros2_pkgs/control/` | 已完成 |
| 4 | 控制与传感器组合入口、状态快照 IPC、整机实时验收 | 未迁移 | 停止 |

## 当前包

`colcon list` 可识别 5 个 ROS 2 包：

- `isaacsim_bringup`
- `isaacsim_command_controller`
- `isaacsim_description`
- `isaacsim_embodiment_contract`
- `isaacsim_sensors`

`ros2_pkgs` 下的目录和包名已去除旧仓库前缀。机器人和传感器仍通过兼容的 ROS 2 topic、
frame 和外部依赖交互，但不再作为多个本地 Git 仓库维护。

## 验证证据

| 检查 | 结果 |
| --- | --- |
| 迁移布局测试 `test/migration/test_stages_1_to_3_layout.py` | 通过 |
| ROS 2 集成/单元测试集合 | `74 passed, 1 warning` |
| 隔离 colcon 构建 | 5 个包通过 |
| `ros2_pkgs` 路径命名检查 | 未发现旧仓库前缀目录/包路径 |
| 阶段 4 全身入口检查 | 未发现，符合停止要求 |

构建产物使用临时目录验证，避免把 `build/`、`install/`、`log/` 写入仓库：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --build-base /tmp/isaacsim_robot_build \
  --install-base /tmp/isaacsim_robot_install \
  --log-base /tmp/isaacsim_robot_log
```

## 控制与传感器边界

- 控制层维护 `controller_manager` 配置、URDF 生成、动作/观测合同和运行验证。
- 传感器层维护 RealSense/MID360 的 rig、ROS 2 适配、QoS/队列/诊断和独立场景。
- 控制回调不负责传感器渲染、点云转换、序列化和写盘。
- 传感器测试通过不代表机器人控制在相同进程的实时 deadline 已通过；该结论见
  [`../sensors/sensor_validation.md`](../sensors/sensor_validation.md)。

## 未完成和风险

1. 控制合同配置仍保留历史运行时兼容字段，后续如切换 Isaac Sim 版本必须同步更新
   `embodiment.yaml`、validator 和对应测试，不能只改 README。
2. RealSense nominal 标定不是实机标定；真实设备接入前要替换内参、畸变、外参和 depth scale。
3. MID360 的机器人动态外参、运动轨迹 ATE/RPE 和 full profile 的高频 CustomMsg 仍未达标。
4. 阶段 4 需要先定义跨进程状态快照/IPC，再做控制无传感器基线、传感器增量和整机验收。

本报告不宣称整机启动、整机实时性能或控制+传感器组合已经完成。
