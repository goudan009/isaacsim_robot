# 独立传感器与双进程相机副本评测

本报告只汇总独立 RealSense、MID360 与“完整机器人控制 + 非权威相机副本”的验证结果。
原始 JSON、启动日志和调试重试均保留在 `raw/<批次编号>/`。双进程副本不拥有物理、控制或
`/clock`，仅用于阶段 5 前的架构性能实验；阶段 4 的正式产品化入口仍未宣称完成。

格式、判定规则和后续 agent 写入流程见
[`../../docs/REPORTING_CN.md`](../../docs/REPORTING_CN.md)。

## 当前结论（2026-09-11）

当前采用两条不同的 MID360 测评线，禁止混用：

- 三进程整机联合线：完整机器人 + 四路 RGB-D + MID360 时，非空 PointCloud2 墙钟接收频率不低于 `8 Hz`。
- 独立 MID360 历史性能线：不加载机器人和相机时，仍按非空输出不低于 `10 Hz` 记录和判定。

最新有效三进程批次为
`20260911-233603-双进程整机四相机-phase5-candidate-cpu64-native-no-object-map`：主控制
`/joint_states=89.998 Hz`、主 physics=`196.457 Hz`、主 RTF=`1.637`；八路 RGB-D
图像为 `20.024–20.622 Hz`；MID360 原始 PointCloud2 墙钟接收为 `9.922 Hz`，
非空样本 `198`，末帧 `58979` 点；六个控制器均为 `active`，底盘、升降、头部和双臂均响应。
因此本批次满足当前联合最低采集线，结论为“通过”，阶段 5 **可进入测试验收**。
该结论表示性能候选方案已达到阶段 5 的入口条件，不等于正式产品化组合已经验收完成；
阶段 5 仍需验收正式启动链路、动态雷达外参同步、时间一致性和重复运行稳定性。

## 阶段 5 入口批次（2026-09-11）

### 批次：20260911-233603-双进程整机四相机-phase5-candidate-cpu64-native-no-object-map

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 23:36:03 +08:00 |
| 执行者 | `phase5-candidate-cpu64-native-no-object-map` |
| 版本 | Isaac Sim 6.0；控制权威、相机副本、雷达副本三进程隔离 |
| 测试范围 | 完整机器人控制；底盘、升降、头部、左右双臂；四路 RGB-D；MID360；无 RViz |
| 配置 | 控制 `physics/render=120/15 Hz`；相机副本 `physics/render=45/30 Hz`；雷达副本 `physics/render=30/30 Hz`；四路 `640×480` RGB-D；相机副本纯视觉运动学、MinimalRendering、关闭逐传感器 TLAS、关闭副本碰撞、隐藏 guide 网格、启用视觉 LOD、关闭 CameraInfo；MID360 `MID360_PERFORMANCE`、native/direct、仿真输出 `10 Hz`、关闭 object-id map；CPU=`0-15 / 16-31 / 32-47` |
| 功能结论 | 通过 |
| 实时结论 | 通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | `89.998 Hz` | 通过 |
| 主进程 physics step rate | 不低于 `120 Hz` | `196.457 Hz` | 通过 |
| 主进程 RTF | 不低于 `1.0` | `1.637` | 通过 |
| 控制器状态 | 6 个均为 `active` | `6/6 active` | 通过 |
| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 控制组 `4/4`；底盘位移 `0.718 m` | 通过 |
| 八路 ROS 图像 | 每路不低于 `20 Hz` | `20.024–20.622 Hz` | 通过 |
| MID360 PointCloud2 | 不低于 `8 Hz` 且非空 | `9.922 Hz`；非空 `198`；末帧 `58979` 点 | 通过 |
| 状态快照施加 | 必须实际施加关节状态 | `1685` 次更新；`48865` 个关节状态 | 通过 |
| 雷达与权威时钟偏差 | 必须记录 P99/max | P99=`8.33 ms`；max=`8.33 ms` | 仅记录 |

结论：本批次验证了当前高性能三进程入口达到阶段 5 的性能入口线。控制权威进程保持
90 Hz 控制状态发布和实时物理，视觉副本八路图像全部达到 20 Hz，独立雷达达到用户确认的
8 Hz 可用线，且状态快照确实施加到 29 个运动学关节。该方案仍是阶段 5 的候选架构，
后续验收不得把独立雷达场景的静态位姿同步误写成产品化动态外参同步。

原始证据：`raw/20260911-233603-双进程整机四相机-phase5-candidate-cpu64-native-no-object-map/`。

## 2026-09-11 最新双进程复测

本节是当前可复核的最新结果。测试沿用历史双进程高性能方案：权威控制进程使用
`physics/render=120/30 Hz`，视觉副本使用纯视觉运动学链、CPU 隔离和 ROS 2 队列线程；
四路相机为 `640×480` RGB-D，目标采集频率为每路不低于 `20 Hz`。完整融合测试在相机
基线通过后执行，未因单项未达标而修改控制链或伪造结果。

| 测试批次 | 功能结论 | 实时结论 | 关键结果 |
| --- | --- | --- | --- |
| `20260911-145042-双进程整机四相机-robot-camera-baseline-20260911` | 通过 | 通过 | `/joint_states=89.998 Hz`；主 physics=`132.632 Hz`；主 RTF=`1.105`；八路图像=`25.540–27.085 Hz`；29 个关节状态已同步并实际施加；底盘、升降、头部、左右臂均响应 |
| `20260911-150521-双进程整机四相机-robot-camera-lidar-fusion-native-20260911` | 通过 | 未通过 | `/joint_states=90.012 Hz`；主 physics=`130.536 Hz`；主 RTF=`1.088`；八路图像=`21.528–23.131 Hz`；MID360=`5.283 Hz`，非空；控制和状态同步正常 |
| `20260911-151311-双进程整机四相机-robot-camera-lidar-fusion-native-no-tlas-20260911` | 通过 | 未通过 | 关闭逐传感器 TLAS 后，八路图像最低=`19.774 Hz`；MID360=`5.070 Hz`；未产生有效优化 |
| `20260911-151747-双进程整机四相机-robot-camera-lidar-fusion-native-minimal-20260911` | 通过 | 未通过 | `MinimalRendering` 后八路图像最低=`19.724 Hz`；MID360=`5.162 Hz`；未产生有效优化 |

以上四个批次是“相机与雷达同属一个视觉副本”架构的历史优化过程。其雷达约 `5 Hz` 的
结果不能代表当前三进程隔离方案；当前联合结论以本报告后面的
`20260911-203737-双进程整机四相机-robot-camera-lidar-physics30-20260911` 为准。

原始证据均保存在 `reports/sensors/raw/` 对应批次目录；每个目录包含 `result.json`、启动
日志、relay 指标和副本指标。

## 2026-09-11 三进程隔离雷达优化

在双进程视觉副本中直接加入 MID360 时，雷达与四路 RGB-D 共享同一个 RTX 更新循环，
雷达只能达到约 `5 Hz`。本轮增加了可选的三进程实验入口：

```text
控制权威进程（机器人物理、ROS 2 控制、/clock）
        ├── 动态相机进程（机器人视觉副本、四路 RGB-D、状态快照）
        └── 独立雷达进程（历史 mid360_empty_stage.usda、MID360 native/direct）
```

独立雷达进程不再加载完整机器人 USD，因为当前测试没有向雷达进程同步机器人状态；
继续加载机器人只会增加 RTX/USD 负载而不带来动态外参收益。雷达进程恢复历史独立仓库
验证过的 `SimulationManager.setup_simulation()` + `simulation_app.update()` 驱动方式，
并将雷达渲染频率设为 `30 Hz`，输出仍为 `10 Hz`。

### 批次：20260911-174604-双进程整机四相机-robot-camera-lidar-fusion-lidar-render30-20260911

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 17:46:04 +08:00 |
| 执行者 | `robot-camera-lidar-fusion-lidar-render30-20260911` |
| 版本 | Isaac Sim 6.0；三进程隔离雷达优化入口 |
| 测试范围 | 完整机器人控制；动态四路 RGB-D；独立轻量 MID360；无 RViz |
| 配置 | 控制 `physics/render=120/15 Hz`；相机副本 `physics/render=45/30 Hz`、相机 `640×480 @ 30 Hz`；雷达独立进程 `physics/render=90/30 Hz`；`MID360_PERFORMANCE`、native/direct、10 Hz；CPU=`0-31 / 32-47 / 48-63` |
| 功能结论 | 通过 |
| 实时结论 | 通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | `90.005 Hz` | 通过 |
| 主进程 physics step rate | 不低于 `120 Hz` | `166.330 Hz` | 通过 |
| 主进程 RTF | 不低于 `1.0` | `1.386` | 通过 |
| 四路 RGB-D 最低墙钟频率 | 每路不低于 `20 Hz` | `21.473–22.322 Hz` | 通过 |
| MID360 PointCloud2 | 不低于 `10 Hz` 且非空 | `10.040 Hz`；非空 `201`；末帧 `29,327` 点 | 通过 |
| 雷达进程 physics / RTF | 仅记录 | `29.996 Hz` / `0.333` | 仅记录 |
| 控制组响应 | 底盘、升降、头部、双臂均响应 | `4/4`；底盘响应 | 通过 |

结论：该配置首次同时满足当前联合准入线：完整机器人控制保持 `90 Hz` 状态发布和
`RTF>1`，四路 RGB-D 均超过 `20 Hz`，MID360 非空 PointCloud2 达到 `10 Hz`。这是
阶段 4 前的性能架构候选，不代表动态雷达外参已经跨进程同步；正式产品化前仍需单独验收
雷达安装位姿同步、时间一致性和传感器组合启动链路。

原始证据：`raw/20260911-174604-双进程整机四相机-robot-camera-lidar-fusion-lidar-render30-20260911/result.json`、
`raw/20260911-174604-双进程整机四相机-robot-camera-lidar-fusion-lidar-render30-20260911/lidar_replica/metrics.json`、
`raw/20260911-174604-双进程整机四相机-robot-camera-lidar-fusion-lidar-render30-20260911/lidar_replica.launch.log`。

### 批次：20260911-203737-双进程整机四相机-robot-camera-lidar-physics30-20260911

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-11 20:37:37 +08:00 |
| 执行者 | `robot-camera-lidar-physics30-20260911` |
| 版本 | Isaac Sim 6.0；控制权威、相机副本、雷达副本三进程隔离 |
| 测试范围 | 完整机器人控制；底盘、升降、头部、左右双臂；四路 RGB-D；MID360；无 RViz |
| 配置 | 控制 `physics/render=120/15 Hz`；相机副本 `physics/render=45/30 Hz`；雷达副本 `physics/render=30/30 Hz`；相机 `640×480`；`MID360_PERFORMANCE`、native/direct、仿真输出 `10 Hz`；CPU=`0-15 / 16-31 / 64-95` |
| 功能结论 | 通过 |
| 实时结论 | 条件通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | `90.001 Hz` | 通过 |
| 主进程 physics step rate | 不低于 `120 Hz` | `193.817 Hz` | 通过 |
| 主进程 RTF | 不低于 `1.0` | `1.615` | 通过 |
| 四路 RGB-D 最低墙钟频率 | 每路不低于 `20 Hz` | `20.308–21.503 Hz` | 通过 |
| MID360 原始 PointCloud2 | 不低于 `8 Hz` 且非空 | `9.822 Hz`；非空 `295`；末帧 `59080` 点 | 通过 |
| 雷达进程 physics / RTF | 仅记录 | `30.003 Hz` / `1.0001` | 仅记录 |
| 控制组响应 | 底盘、升降、头部、双臂均响应 | `4/4`；底盘位移 `0.722 m` | 通过 |
| 雷达动态位移 | 必须记录 | `0.722 m` | 仅记录 |
| 雷达与权威时钟偏差 | 必须记录 P99/max | P99=`8.33 ms`；max=`16.67 ms` | 仅记录 |
| 雷达状态快照时延 | 必须记录 P99/max | P99=`16.74 ms`；max=`3.258 s` | 仅记录 |

结论：三进程隔离方案满足当前联合最低采集线，雷达非空点云约 `9.822 Hz`，高于用户确认的
`8 Hz` 可用线；控制和四路相机也满足本批次通过线。该批次已满足阶段 5 入口性能条件，
但因状态快照最大时延存在秒级离群值，动态外参与正式组合启动链路仍未完成产品化验收，
故该历史批次本身只记为“条件通过”，不覆盖后续阶段 5 入口批次的“通过”结论。

补充：本批次的 `lidar_restamper.metrics.json` 原始版本中，`output_wall_hz=5.959 Hz`
是从重打包进程启动到结束的总墙钟统计，包含等待首条输入消息的启动阶段，不能替代整机
订阅窗口的 `9.822 Hz`。后续版本同时记录 `input_active_wall_hz` 和 `output_active_wall_hz`，
报告判定应使用有效消息窗口频率。

原始证据：`raw/20260911-203737-双进程整机四相机-robot-camera-lidar-physics30-20260911/`。

1. `640×480`、逐帧 CameraInfo 开启时，当前最低采集线已通过。历史基线批次
   `20260909-220000-双进程整机四相机-codex-kinematic-v5` 的八路 ROS 图像为
   `24.584–25.934 Hz`。
2. 新的“相机 40 Hz、视觉副本姿态 30 Hz”候选批次
   `20260909-223103-双进程整机四相机-codex-kinematic-sync30-tick40` 为
   `24.841–27.160 Hz`，平均值较 `v5` 提升约 `4.2%`；仍未达到每路 `29 Hz`。
3. 两个批次的主控制权威进程均通过：`/joint_states` 约 `90 Hz`、physics step rate
   超过 `120 Hz`、RTF 超过 `1.0`，并验证了底盘、升降、头部、左右臂响应。
4. 2026-09-09 的“4 路 RGB-D + MID360”联合批次 `20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5` 中，主控制仍通过（`/joint_states=90.003 Hz`、physics=`124.696 Hz`、RTF=`1.039`），八路图像为 `22.021–23.727 Hz`，但 MID360 PointCloud2 仅 `1.686 Hz`，未达到 `10 Hz` 测评线；并且副本未提取到关节链，不能作为动态整机验收。
5. 独立静态场景的四台物理 RGB-D 可达到八路约 `30 Hz`。瓶颈只在完整机器人动态视觉副本中
出现，当前证据指向动态 USD/RTX/ROS 图像链路，而非单纯的像素数或三角面数量。

| 模块 | 功能结论 | 实时结论 | 当前可用范围 |
| --- | --- | --- | --- |
| 独立 RealSense 四物理相机 | 通过 | 条件通过 | 静态场景、八路约 `30 Hz`；同进程控制尾延迟未通过 |
| 三进程整机四相机副本 + MID360 | 通过 | 通过 | `640×480`、八路每路 `20.024–20.622 Hz`；MID360 `9.922 Hz`，联合线为 `8 Hz` |
| MID360 性能档 | 条件通过 | 条件通过 | 独立历史线仍按 `10 Hz`；三进程整机联合线按 `8 Hz`，最新为 `9.922 Hz` |
| MID360 近似档 | 条件通过 | 条件通过 | 原始点云约 `10 Hz`；CustomMsg 约 `3–5 Hz`，不可混写 |
| 控制与传感器产品化组合（阶段 4） | 待验收 | 待验收 | 阶段 5 入口已通过，正式产品化链路未验收 |

## 测评线与推荐配置

| 项目 | 通过线 | 当前结论 |
| --- | --- | --- |
| 主控制 `/joint_states` | 不低于 `90 Hz` | 维持 `90 Hz` |
| 主控制 physics step rate | 不低于 `120 Hz` | 维持 `120 Hz` 配置并实测 |
| 主控制 RTF | 不低于 `1.0` | 必须与机器人响应一同记录 |
| 副本最低采集 | 八路 RGB/Depth 每路不低于 `20 Hz` | `v5` 与 30 Hz 姿态候选均通过 |
| 副本正式目标 | 八路 RGB/Depth 每路不低于 `29 Hz` | 未通过；不得写成 `30 Hz` |
| 副本状态同步 | 收到且实际施加；记录 P99/max | 两个有效批次 P99 约 `19–20 ms`，均存在秒级 max 离群值 |
| 相机 + MID360 联合准入 | 主控制达标、八路图像≥`20 Hz`、MID360≥`8 Hz`、动态状态与时钟证据完整 | 通过；阶段 5 已进入验收 |
| 阶段 4 验收 | 状态、对象、时间一致性与产品化启动链路完整 | 停止，不可由本报告替代 |

推荐使用当前三进程入口批次作为 `20 Hz` 最低采集基线；若任务能接受视觉副本仅以 `30 Hz`
更新机器人姿态，可使用 `sync30-tick40` 作为更高吞吐的实验候选。两者都不是阶段 4 产品化方案。

## 历史独立仓库复核与当前合并状态

历史独立仓库为 `../openflex_isaac_sensor_assets`，复核版本为 `a2a4bf9`
（标签：`isaacsim-6.0-mid360`）。该仓库的 MID360 方案本身没有发现功能性缺陷：使用
`Example_Rotary`、`Lidar.create(accumulate_outputs=True, aux_output_level="FULL", tick_rate=10)`、
`LidarSensor(annotators=["generic-model-output"])` 和 `ROS2RtxLidarHelper`，历史旧 REST 基线已
记录 raw PointCloud2 仿真时间频率 `10 Hz`、`100/100` 非空消息、平均约 `34,085` 点/帧；兼容
`CustomMsg` 也记录为约 `10 Hz` 仿真时间频率和 `97` 条非空消息。

已将该方案合并到 `ros2_pkgs/simulation_bridge/sensor_pkg/isaacsim_sensors/mid360.py`，独立场景
也已合并到 `isaac_sim_core/assets/environments/mid360_empty_stage.usda`。合并前发现一处偏差：
独立 `direct` 图额外插入了 `OgnIsaacRunOneSimulationFrame`；历史通过图是
`OnPlaybackTick → ROS2RtxLidarHelper` 直连。该偏差已修复，机器人侧 `helper` 也复用同一历史图，
并新增静态回归测试。

当前状态：修复后的真实独立 MID360 和“机器人 + MID360（无相机）”历史方案已完成复核；
最新三进程整机入口已通过阶段 5 性能入口线，后续只新增阶段 5 验收批次，不再重复创建结论文档。
原因是 GPU 上仍有用户启动的 Isaac Sim 进程，测试脚本已主动阻止并发启动，避免把 GPU 争用误记为
雷达性能结论；旧的并发失败批次不作为修复后的结论。

## MID360 独立历史链路复测

本节只记录“不加载机器人、相机、控制或 RViz”的静态 MID360 测试。复测固定保留历史链路：
`Example_Rotary`、`MID360_PERFORMANCE`、RTX `ROS2RtxLidarHelper` 原始
`/openflex/livox_frame/lidar`，以及可选的兼容桥 `/livox/lidar` CustomMsg。

原始 PointCloud2 与兼容桥 CustomMsg 的频率必须分别记录；独立 PointCloud2 通过线为仿真时间
`10 Hz` 且非空，performance 档 CustomMsg 的独立基线通过线同为 `10 Hz` 且非空。墙钟频率、RTF 和
physics wall Hz 用于判断实时能力，不能替代传感器仿真时间频率。带有其他 Isaac 进程/GPU
负载的批次只能作为并发诊断，不能写成独立性能基线。

### 批次：20260910-002410-独立MID360-codex-mid360-standalone-concurrent

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 00:24:10 +0800 |
| 执行者 | codex-mid360-standalone-concurrent |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=44`；兼容桥=开启 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=不适用 | 未通过 |
| CustomMsg 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 未采集 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 未采集 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-002410-独立MID360-codex-mid360-standalone-concurrent/result.json`、`raw/20260910-002410-独立MID360-codex-mid360-standalone-concurrent/isaac.launch.log`、`raw/20260910-002410-独立MID360-codex-mid360-standalone-concurrent/compat_bridge.log`、`raw/20260910-002410-独立MID360-codex-mid360-standalone-concurrent/isaac/metrics.json`。
### 批次：20260910-002841-独立MID360-codex-mid360-standalone-concurrent-v2

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 00:28:41 +0800 |
| 执行者 | codex-mid360-standalone-concurrent-v2 |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=44`；兼容桥=开启 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=不适用 | 未通过 |
| CustomMsg 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 90.002 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-002841-独立MID360-codex-mid360-standalone-concurrent-v2/result.json`、`raw/20260910-002841-独立MID360-codex-mid360-standalone-concurrent-v2/isaac.launch.log`、`raw/20260910-002841-独立MID360-codex-mid360-standalone-concurrent-v2/compat_bridge.log`、`raw/20260910-002841-独立MID360-codex-mid360-standalone-concurrent-v2/isaac/metrics.json`。
### 批次：20260910-010252-独立MID360-codex-mid360-target-array-smoke

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 01:02:52 +0800 |
| 执行者 | codex-mid360-target-array-smoke |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=51`；兼容桥=关闭 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 90.000 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-010252-独立MID360-codex-mid360-target-array-smoke/result.json`、`raw/20260910-010252-独立MID360-codex-mid360-target-array-smoke/isaac.launch.log`、`raw/20260910-010252-独立MID360-codex-mid360-target-array-smoke/compat_bridge.log`、`raw/20260910-010252-独立MID360-codex-mid360-target-array-smoke/isaac/metrics.json`。
### 批次：20260910-010601-独立MID360-codex-mid360-srtx-off-smoke

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 01:06:01 +0800 |
| 执行者 | codex-mid360-srtx-off-smoke |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=52`；兼容桥=关闭 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 89.999 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-010601-独立MID360-codex-mid360-srtx-off-smoke/result.json`、`raw/20260910-010601-独立MID360-codex-mid360-srtx-off-smoke/isaac.launch.log`、`raw/20260910-010601-独立MID360-codex-mid360-srtx-off-smoke/compat_bridge.log`、`raw/20260910-010601-独立MID360-codex-mid360-srtx-off-smoke/isaac/metrics.json`。
### 批次：20260910-010715-独立MID360-codex-mid360-single-gpu-smoke

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 01:07:15 +0800 |
| 执行者 | codex-mid360-single-gpu-smoke |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=53`；兼容桥=关闭 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 90.000 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-010715-独立MID360-codex-mid360-single-gpu-smoke/result.json`、`raw/20260910-010715-独立MID360-codex-mid360-single-gpu-smoke/isaac.launch.log`、`raw/20260910-010715-独立MID360-codex-mid360-single-gpu-smoke/compat_bridge.log`、`raw/20260910-010715-独立MID360-codex-mid360-single-gpu-smoke/isaac/metrics.json`。
### 批次：20260910-011222-独立MID360-codex-mid360-direct-sensor-smoke

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 01:12:22 +0800 |
| 执行者 | codex-mid360-direct-sensor-smoke |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=55`；兼容桥=关闭 |
| 并发 Isaac 负载 | 有（本批次只作并发诊断，不作独立实时基线） |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 89.999 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。检测到其他 Isaac 进程，本批次不得作为独立性能基线。

原始证据：`raw/20260910-011222-独立MID360-codex-mid360-direct-sensor-smoke/result.json`、`raw/20260910-011222-独立MID360-codex-mid360-direct-sensor-smoke/isaac.launch.log`、`raw/20260910-011222-独立MID360-codex-mid360-direct-sensor-smoke/compat_bridge.log`、`raw/20260910-011222-独立MID360-codex-mid360-direct-sensor-smoke/isaac/metrics.json`。
### 批次：20260910-085311-独立MID360-codex-mid360-standalone-final

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 08:53:11 +0800 |
| 执行者 | codex-mid360-standalone-final |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=44`；兼容桥=关闭 |
| 并发 Isaac 负载 | 无 |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 89.999 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。本批次可与历史独立链路做同口径比较。

原始证据：`raw/20260910-085311-独立MID360-codex-mid360-standalone-final/result.json`、`raw/20260910-085311-独立MID360-codex-mid360-standalone-final/isaac.launch.log`、`raw/20260910-085311-独立MID360-codex-mid360-standalone-final/compat_bridge.log`、`raw/20260910-085311-独立MID360-codex-mid360-standalone-final/isaac/metrics.json`。
### 批次：20260910-095252-独立MID360-direct-history-retest

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-10 09:52:52 +0800 |
| 执行者 | direct-history-retest |
| 版本 | Isaac Sim 6.0；当前仓库的历史独立 MID360 等效链路 |
| 测试范围 | 静态 `mid360_empty_stage.usda`；不加载机器人、相机、ROS 2 控制或 RViz |
| 配置 | `MID360_PERFORMANCE`；`Example_Rotary`；physics/render `90/60 Hz`；headless；`ROS_DOMAIN_ID=53`；兼容桥=关闭 |
| 并发 Isaac 负载 | 无 |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| raw PointCloud2 仿真时间频率 | ≥`10 Hz` 且非空 | 未采集 Hz；非空=未采集 | 未通过 |
| raw PointCloud2 墙钟频率 | 仅记录 | 未采集 Hz | 仅记录 |
| raw 点数/帧（平均 / 范围） | performance 约`30k`；full 约`52.8k`，随命中率波动 | 未采集 / 未采集..未采集 | 仅记录 |
| CustomMsg 仿真时间频率 | 未启用 | 不适用 Hz；非空=不适用 | 通过 |
| CustomMsg 墙钟频率 | 仅记录 | 不适用 Hz | 仅记录 |
| physics wall Hz | 仅记录 | 89.999 Hz | 仅记录 |
| RTF | 独立实时基线≥`1.0` | 1.000 | 未通过 |

结论：raw PointCloud2 与 `/livox/lidar` CustomMsg 是不同链路，频率不可互相替代。本批次可与历史独立链路做同口径比较。

原始证据：`raw/20260910-095252-独立MID360-direct-history-retest/result.json`、`raw/20260910-095252-独立MID360-direct-history-retest/isaac.launch.log`、`raw/20260910-095252-独立MID360-direct-history-retest/compat_bridge.log`、`raw/20260910-095252-独立MID360-direct-history-retest/isaac/metrics.json`。
<!-- MID360_STANDALONE_BATCHES -->

## 关键有效批次

### 批次：20260909-220000-双进程整机四相机-codex-kinematic-v5

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 22:00:00 +08:00 |
| 执行者 | codex |
| 版本 | Isaac Sim 6.0；双 Isaac 进程实验入口 |
| 测试范围 | 主进程：完整机器人、物理、ROS 2 控制、无相机；副本：纯视觉运动学机器人、四台 RGB-D、只读状态同步；headless、无 RViz、无雷达 |
| 配置 | 主进程 physics/render `120/30 Hz`、CPU `0-31`；副本 physics/render `55/30 Hz`、CPU `32-63`；四台 `640×480 @ 40 Hz` RGB-D；逐帧 CameraInfo、逐传感器 TLAS、异步 ROS 2 发布队列开启；SRTX 关闭 |
| 功能结论 | 条件通过（最低采集线） |
| 实时结论 | 条件通过（仅主控制权威进程） |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | `90.015 Hz` | 通过 |
| 主进程 physics step rate | 不低于 `120 Hz` | `126.230 Hz` | 通过 |
| 主进程 RTF | 不低于 `1.0` | `1.052` | 通过 |
| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 控制组 `4/4`；底盘 `0.719 m` | 通过 |
| 八路 ROS 图像 | 每路不低于 `20 Hz` | `24.584–25.934 Hz` | 通过 |
| 状态快照 | 收到且施加 | `719/719`；29 joints | 通过 |
| 快照时延 P99 / max | 记录 | `19.333 / 1282.980 ms` | 仅记录 |
| 副本 physics RTF | 记录 | `0.732` | 仅记录 |

原始证据：`raw/20260909-220000-双进程整机四相机-codex-kinematic-v5/`。

### 批次：20260909-223103-双进程整机四相机-codex-kinematic-sync30-tick40

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 22:31:03 +08:00 |
| 执行者 | codex |
| 测试范围 | 与 `v5` 相同；副本相机仍为 `640×480 @ 40 Hz`，仅将机器人视觉姿态施加限为 `30 Hz` |
| 配置 | 主进程 physics/render `120/30 Hz`；副本 physics/render `55/30 Hz`；逐帧 CameraInfo、TLAS、异步 ROS 2 发布队列开启；SRTX 关闭 |
| 功能结论 | 条件通过（最低采集线） |
| 实时结论 | 条件通过（仅主控制权威进程） |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | `90.014 Hz` | 通过 |
| 主进程 physics step rate | 不低于 `120 Hz` | `127.824 Hz` | 通过 |
| 主进程 RTF | 不低于 `1.0` | `1.065` | 通过 |
| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 控制组 `4/4`；底盘 `0.721 m` | 通过 |
| 八路 ROS 图像 | 每路不低于 `20 Hz` | `24.841–27.160 Hz` | 通过 |
| 状态快照 | 收到且施加 | 收到 `945`；施加 `567`；29 joints | 通过 |
| 快照时延 P99 / max | 记录 | `20.177 / 1389.914 ms` | 仅记录 |
| 副本 physics RTF | 记录 | `0.756` | 仅记录 |

结论：仅减少视觉副本的姿态写入，将平均图像频率从 `25.491 Hz` 提升至 `26.565 Hz`；但最慢
一路仍为 `24.841 Hz`，未达到 `29 Hz`。该模式降低了视觉副本姿态时间分辨率，需由采集任务
确认可接受后才能使用。

原始证据：`raw/20260909-223103-双进程整机四相机-codex-kinematic-sync30-tick40/`。

### 批次：20260909-160015-独立相机-quad-codex

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 16:00:15 +08:00 |
| 执行者 | codex |
| 测试范围 | 独立静态场景；2 台 D435 + 2 台 D405；不加载机器人、不启动控制或阶段 4 集成 |
| 配置 | physics/render `90/90 Hz`；四台 `640×480 @ 30 Hz` RGB-D；headless；Reliable QoS；Kit threads `16` |
| 功能结论 | 通过 |
| 实时结论 | 条件通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| RenderProduct | `4` 且唯一 | `4` | 通过 |
| 八路 ROS 图像 | 每路约 `30 Hz` | `30.133–30.143 Hz` | 通过 |
| RTF | 不低于 `1.0` | `0.999690` | 条件通过（约等于 1） |
| physics gap P99 / max | P99≤`13.333 ms`；max≤`22.222 ms` | `25.011 / 32.497 ms` | 未通过 |

原始证据：`raw/20260909-160015-独立相机-quad-codex/`。

## 已排除的优化实验

| 批次 | 变更 | 八路图像 Hz | 结论 |
| --- | --- | ---: | --- |
| `20260909-220532-双进程整机四相机-codex-kinematic-noinfo` | `640×480`，关闭逐帧 CameraInfo | `25.275–27.100` | 略快，但输出不完整 |
| `20260909-220245-双进程整机四相机-codex-kinematic-aa0` | 关闭 AA | `24.512–26.120` | 无有效增益 |
| `20260909-220805-双进程整机四相机-codex-kinematic-notlas` | 关闭逐传感器 TLAS | `23.848–25.709` | 性能下降 |
| `20260909-221041-双进程整机四相机-codex-kinematic-480p` | `480×360`，CameraInfo 开启 | `26.870–27.817` | 未达到 `29 Hz` |
| `20260909-221511-双进程整机四相机-codex-kinematic-424x240` | `424×240`，CameraInfo 开启 | `26.019–26.661` | 像素量不是主瓶颈 |
| `20260909-221818-双进程整机四相机-codex-kinematic-sync30` | 相机 tick 也改为 `30 Hz` | `17.585–19.377` | 性能下降，不能用 |
| `20260909-222116-双进程整机四相机-codex-kinematic-hide-guide` | 隐藏 27 个 `guide` 碰撞网格 | `23.280–25.836` | 无有效增益 |
| `20260909-222714-双进程整机四相机-codex-kinematic-lod-proxy` | 38 个 link 包围盒视觉 LOD | `23.946–25.347` | 无有效增益 |
| `20260909-165422-双进程整机四相机-codex` | SRTX transport | 无图像输出 | `127.0.0.1:8081` 后端未运行 |

除 30 Hz 姿态写入的小幅提升外，已验证的开关均不能达到 `29 Hz`。后续若必须达到正式目标，
需要验证架构级方案：将 link pose 更新从 Python/USD 热路径迁移至原生/编译式实现，或使用真实
离线制作并完成几何保真验证的视觉 LOD 资产。

## 配置位置与复测入口

| 项目 | 唯一配置位置 |
| --- | --- |
| RealSense 独立配置 | `isaac_sim_core/config/sensor_params/realsense/realsense_standalone.yaml` |
| RealSense 机器人挂载 | `isaac_sim_core/config/sensor_params/realsense/realsense_robot_mounts.yaml` |
| RealSense 标定 | `isaac_sim_core/config/sensor_params/realsense/calibration/` |
| MID360 机器人挂载 | `isaac_sim_core/config/sensor_params/mid360/mid360_robot_mount.yaml` |
| MID360 语义映射 | `isaac_sim_core/config/sensor_params/mid360/mid360_semantic_mapper.yaml` |

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAACSIM_PATH=/home/1024201092WYH/isaacsim-6.0
python3 test/performance/robot_control_isolated_camera_performance.py \
  --replica-physics-hz 55 --replica-render-hz 30 --camera-tick-rate-hz 40 \
  --camera-width 640 --camera-height 480 --visual-kinematic-replica \
  --kinematic-sync-hz 30 --publish-with-queue-thread --min-camera-hz 20
```

复测只能新增原始批次目录；确认后的结论更新本报告的“当前结论”或“已排除的优化实验”表。

### 批次：20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 23:47:56 +0800 |
| 执行者 | codex-camera-lidar-environment-v3 |
| 版本 | Isaac Sim 6.0；双 Isaac 进程实验入口 |
| 测试范围 | 主进程：完整机器人、物理、ROS 2 控制、无传感器；副本进程：机器人视觉副本、四台 RGB-D、MID360、只读状态同步；无 RViz |
| 配置 | 主进程 physics/render `120.0/30.0 Hz`、CPU `0-31`；副本 physics/render `55.0/30.0 Hz`、CPU `32-63`；四台 `640×480 @ 40.0 Hz` RGB-D；MID360=`MID360_PERFORMANCE`、transport=`helper`；副本模式=`纯视觉运动学`；运动学状态施加=`30.0`；副本碰撞=`开启`；guide 碰撞网格隐藏=`False`；视觉 LOD 代理=`False`；MinimalRendering=`False`；逐传感器 TLAS=`True`；AA=`None`；SRTX=`False`；发布队列线程=`True`；逐帧 CameraInfo=`True`；UDP `127.0.0.1:24103` |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | 90.001 Hz | 通过 |
| 主进程 physics step rate | 不低于 `120.0 Hz` | 123.727 Hz | 通过 |
| 主进程 RTF | 不低于 `1.0` | 1.031 | 通过 |
| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 底盘=0.720 m；控制组=4/4 | 通过 |
| 副本 RenderProduct 数 | `4` | 4 | 通过 |
| 副本 ROS 图像实收频率 | 每路不低于 `20.0 Hz` | 21.897–24.349 Hz | 通过 |
| MID360 PointCloud2 | 不低于 `10.0 Hz` 且非空 | 1.143 Hz；非空=46；末帧点数=93814 | 未通过 |
| UDP 状态快照 | 有接收并施加关节状态 | 收=1421；施加=815；关节=0；relay=139.024 Hz | 未通过 |
| 快照时延 P99 / max | 仅记录 | 20.858 / 27.617 ms | 仅记录 |

结论：该批次的八路图像和主控制通过，但 MID360 仅 `1.143 Hz`，且专用场景重定义了机器人已有挂载 prim，导致副本没有提取到关节链（实际施加关节数为 `0`）。因此本批次不能作为动态整机传感器验收；它只证明专用环境可产生非空点云。双进程副本即使后续通过，也只是阶段 4 前的架构验证，不是产品化传感器集成验收。

原始证据：`raw/20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3/result.json`、`raw/20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3/authority.launch.log`、`raw/20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3/replica.launch.log`、`raw/20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3/relay.metrics.json`、`raw/20260909-234756-双进程整机四相机-codex-camera-lidar-environment-v3/replica/metrics.json`。

### 批次：20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5

| 字段 | 内容 |
| --- | --- |
| 执行时间 | 2026-09-09 23:56:01 +0800 |
| 执行者 | codex-camera-lidar-optimized-v5 |
| 版本 | Isaac Sim 6.0；双 Isaac 进程实验入口 |
| 测试范围 | 主进程：完整机器人、物理、ROS 2 控制、无传感器；副本进程：机器人视觉副本、四台 RGB-D、MID360、只读状态同步；无 RViz |
| 配置 | 主进程 physics/render `120.0/30.0 Hz`、CPU `0-31`；副本 physics/render `55.0/30.0 Hz`、CPU `32-63`；四台 `640×480 @ 40.0 Hz` RGB-D；MID360=`MID360_PERFORMANCE`、transport=`helper`；副本模式=`纯视觉运动学`；运动学状态施加=`30.0`；副本碰撞=`开启`；guide 碰撞网格隐藏=`False`；视觉 LOD 代理=`False`；MinimalRendering=`False`；逐传感器 TLAS=`True`；AA=`None`；SRTX=`False`；发布队列线程=`True`；逐帧 CameraInfo=`True`；UDP `127.0.0.1:24103` |
| 功能结论 | 未通过 |
| 实时结论 | 未通过 |

| 指标 | 通过线 | 实测值 | 判定 |
| --- | --- | ---: | --- |
| 主进程 `/joint_states` | 不低于 `90 Hz` | 90.003 Hz | 通过 |
| 主进程 physics step rate | 不低于 `120.0 Hz` | 124.696 Hz | 通过 |
| 主进程 RTF | 不低于 `1.0` | 1.039 | 通过 |
| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 底盘=0.720 m；控制组=4/4 | 通过 |
| 副本 RenderProduct 数 | `4` | 4 | 通过 |
| 副本 ROS 图像实收频率 | 每路不低于 `20.0 Hz` | 22.021–23.727 Hz | 通过 |
| MID360 PointCloud2 | 不低于 `10.0 Hz` 且非空 | 1.686 Hz；非空=65；末帧点数=93867 | 未通过 |
| UDP 状态快照 | 有接收并施加关节状态 | 收=1391；施加=805；关节=0；relay=139.667 Hz | 未通过 |
| 快照时延 P99 / max | 仅记录 | 21.465 / 2028.837 ms | 仅记录 |

结论：相较 v3，点云从 `1.143 Hz` 提升至 `1.686 Hz`，但仍远低于 `10 Hz`。同时副本关节链仍未提取（实际施加关节数为 `0`），因此该批次不能作为动态整机传感器验收。双进程副本即使后续通过，也只是阶段 4 前的架构验证，不是产品化传感器集成验收。

原始证据：`raw/20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5/result.json`、`raw/20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5/authority.launch.log`、`raw/20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5/replica.launch.log`、`raw/20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5/relay.metrics.json`、`raw/20260909-235601-双进程整机四相机-codex-camera-lidar-optimized-v5/replica/metrics.json`。
