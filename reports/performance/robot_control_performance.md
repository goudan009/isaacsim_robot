# 机器人控制性能报告

本文件是机器人 ROS 2 控制相关性能测试的唯一结论报告。测试原始证据按批次保存在
`reports/performance/raw/robot_control_full/`，每次测试不得新建日期结论文档。

## 当前结论

截至 2026-09-11，完整机器人控制链和 MID360 已完成一次可复现的有效组合验证。当前推荐
MID360 方案为 `fixed_kinematic + helper`；`follow` 仅保留为实验模式，不作为产品配置。

| 测试场景 | 功能结论 | 实时结论 | 关键结果 |
| --- | --- | --- | --- |
| 无传感器完整控制，`render_hz:=30` | 通过 | 通过 | `/joint_states` 约 `90 Hz`，RTF `1.464` |
| 完整控制 + MID360，`fixed_kinematic` | 通过 | 通过 | MID360 `11.685 Hz`，physics `140.338 Hz`，RTF `1.1695` |
| 完整控制 + MID360，`fixed` | 通过 | 通过 | MID360 `12.426 Hz`，RTF `1.2429` |
| 完整控制 + 四路 RGB-D 相机，单进程负载探针，`render_hz:=30` | 未通过 | 未通过 | 每路约 `1.68–1.78 Hz`，physics `7.310 Hz`，RTF `0.061` |
| 完整控制 + 四路 RGB-D + MID360，三进程阶段 5 入口候选 | 通过 | 通过 | `/joint_states=89.998 Hz`；主 physics `196.457 Hz`；主 RTF `1.637`；八路图像 `20.024–20.622 Hz`；MID360 `9.922 Hz` |

阶段 5 的性能入口条件已满足，但正式产品化组装仍待验收。后续必须完成正式启动链路、动态
雷达外参、时间一致性和重复稳定性验证；本报告不把旧的全身启动仓库或未经授权的传感器组合
当作正式产品化结论。完整证据见 [`../sensors/sensor_validation.md`](../sensors/sensor_validation.md)。

## 测评线

| 指标 | 通过线 |
| --- | --- |
| 所需控制器 | 全部为 `active` |
| 底盘响应 | 轮速大于 `0` 且里程计位移大于 `0` |
| 升降、头部、左臂、右臂 | 每组目标关节均产生有效位移 |
| `/joint_states` | 不低于 `90 Hz` |
| physics step rate | 不低于 `120 Hz` |
| RTF | 不低于 `1.0` |
| MID360 `performance` | 非空点云接收频率不低于 `10 Hz` |
| 四路相机探针 | 每路 RGB、Depth 不低于 `29.5 Hz`；当前仅作为负载探针 |

功能结论和实时结论分开判定。`/joint_states` 是墙钟控制链频率，不能替代 physics step rate
或 RTF；传感器消息必须同时满足频率、非空和时间戳单调条件。

## 有效批次记录

### 功能验证（自动追加）

以下标记由 `robot_control_full_performance.py` 使用。只有显式传入
`--append-report` 的有效批次才会追加到此处；失败重试只保留在原始证据目录。

<!-- ROBOT_CONTROL_FULL_BATCHES -->

### 实时性能与底盘验证（自动追加）

以下标记由 `robot_control_performance.py` 使用。该区域集中记录无传感器控制基线、
底盘响应和实时性能复测，不为每次测试创建新的结论文档。

<!-- ROBOT_CONTROL_BASELINE_BATCHES -->

### 批次：20260911-114616-机器人加MID360-continue-fixed-kinematic-20260911-rerun

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 11:46:16 +08:00 |
| 执行者 | `continue-fixed-kinematic-20260911-rerun` |
| 版本 | 未记录 |
| 测试范围 | 完整机器人控制；底盘、升降、头部、左臂、右臂；MID360；headless；无 RViz；无相机 |
| 配置 | physics `120 Hz`；render `30 Hz`；控制目标 `90 Hz`；MID360 `performance`；`helper`；`fixed_kinematic` |
| 功能结论 | 通过 |
| 实时结论 | 通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | ---: | ---: | --- |
| 控制器状态 | 全部 `active` | 6/6 `active` | 通过 |
| 底盘里程计位移 | 大于 `0 m` | `0.3014 m` | 通过 |
| 升降、头部、左臂、右臂 | 全部目标关节移动 | `1/1`、`2/2`、`8/8`、`8/8` | 通过 |
| `/joint_states` | 不低于 `90 Hz` | `90.000 Hz` | 通过 |
| physics 推进速率 | 不低于 `120 Hz` | `140.338 Hz` | 通过 |
| RTF | 不低于 `1.0` | `1.1695` | 通过 |
| MID360 非空点云 | 不低于 `10 Hz` | `11.685 Hz` | 通过 |
| MID360 末帧点数 | 大于 `0` | `59,312` | 通过 |

结论：当前 `fixed_kinematic + helper` 方案满足机器人完整控制、90 Hz 状态发布、实时物理和
MID360 非空点云测评线。相同配置的后续复测继续追加到本文件。

原始证据：
`raw/robot_control_full/20260911-114616-机器人加MID360-continue-fixed-kinematic-20260911-rerun/result.json`、
`raw/robot_control_full/20260911-114616-机器人加MID360-continue-fixed-kinematic-20260911-rerun/launch.log`。

### 批次：20260911-092508-机器人加MID360-continue-fixed-kinematic

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 09:25:08 +08:00 |
| 执行者 | `continue-fixed-kinematic` |
| 测试范围 | 完整机器人控制 + MID360；headless；无 RViz；无相机 |
| 配置 | physics `120 Hz`；render `30 Hz`；控制目标 `90 Hz`；`performance`；`helper`；`fixed_kinematic` |
| 功能结论 | 通过 |
| 实时结论 | 通过 |

| 指标 | 实测值 |
| --- | ---: |
| `/joint_states` | `90.001 Hz` |
| physics 推进速率 | `147.450 Hz` |
| RTF | `1.2287` |
| MID360 非空点云频率 | `12.236 Hz` |
| MID360 非空消息数 | `245` |
| MID360 末帧点数 | `59,290` |
| 控制组 | 底盘、升降、头部、左臂、右臂全部通过 |

原始证据：
`raw/robot_control_full/20260911-092508-机器人加MID360-continue-fixed-kinematic/result.json`、
`raw/robot_control_full/20260911-092508-机器人加MID360-continue-fixed-kinematic/launch.log`。

### 批次：20260911-092931-机器人加MID360-continue-fixed

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 09:29:31 +08:00 |
| 执行者 | `continue-fixed` |
| 测试范围 | 完整机器人控制 + MID360；headless；无 RViz；无相机 |
| 配置 | physics `120 Hz`；render `30 Hz`；控制目标 `90 Hz`；`performance`；`helper`；`fixed` |
| 功能结论 | 通过 |
| 实时结论 | 通过 |

| 指标 | 实测值 |
| --- | ---: |
| `/joint_states` | `90.000 Hz` |
| physics 推进速率 | `149.144 Hz` |
| RTF | `1.2429` |
| MID360 非空点云频率 | `12.426 Hz` |
| MID360 非空消息数 | `248` |
| MID360 末帧点数 | `59,302` |
| 控制组 | 底盘、升降、头部、左臂、右臂全部通过 |

原始证据：
`raw/robot_control_full/20260911-092931-机器人加MID360-continue-fixed/result.json`、
`raw/robot_control_full/20260911-092931-机器人加MID360-continue-fixed/launch.log`。

### 批次：20260909-150626-完整控制-codex

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 15:06:26 +08:00 |
| 执行者 | `codex` |
| 测试范围 | 完整机器人控制 + 四路 RealSense RGB-D；headless；无 RViz；不含雷达和 SLAM |
| 配置 | physics `120 Hz`；render `30 Hz`；控制目标 `90 Hz`；四路 `640×480 @ 30 Hz` |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | ---: | ---: | --- |
| 四路相机创建 | 4 路 | 4 路 | 通过 |
| 每路 RGB/Depth | 不低于 `29.5 Hz` | `1.677–1.779 Hz` | 未通过 |
| `/joint_states` | 不低于 `90 Hz` | `90.000 Hz` | 通过 |
| physics 推进速率 | 不低于 `120 Hz` | `7.310 Hz` | 未通过 |
| RTF | 不低于 `1.0` | `0.061` | 未通过 |

结论：相机已创建并发布，主要瓶颈是四路 RGB-D Render Product 的渲染负载，而不是控制器未启动。
该批次是负载探针，不构成阶段 4 验收通过。

原始证据：`raw/robot_control_full/20260909-150626-完整控制-codex/`。

## 排除和停止的批次

以下批次保留原始证据，但不计入当前有效结论：

| 批次 | 状态 | 原因 |
| --- | --- | --- |
| `20260911-093152-机器人加MID360-continue-follow` | 停止 | `follow` 运行期动态更新 USD 位姿，未产生有效雷达消息；曾触发 RTX 风险 |
| `20260911-112504-机器人加MID360-continue-fixed-kinematic-20260911` | 测试异常 | 启动时 REST/DDS socket 被受限环境拒绝，控制器未发现 |
| 2026-09-10 及更早的 MID360 重试 | 测试异常或未通过 | 历史链路、QoS、启动时序或消息采集问题；以本报告中的有效批次为准 |

## 复测入口

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAAC_PATH=/home/1024201092WYH/isaacsim
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

python3 test/performance/robot_control_full_performance.py \
  --duration 20 \
  --command-seconds 3 \
  --lidar-profile performance \
  --lidar-transport helper \
  --lidar-mount-mode fixed_kinematic \
  --stage "$PWD/isaac_sim_core/assets/environments/mid360_empty_stage.usda" \
  --append-report \
  --executor <执行者>
```

正式报告只追加确认有效的批次；失败重试只保留 `result.json` 和 `launch.log`，不复制成新的结论
文档。
