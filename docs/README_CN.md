# 文档索引

本目录只保留三份长期维护的说明，避免把迁移过程中的旧仓库设计稿当成当前使用说明。

| 文档 | 内容 |
| --- | --- |
| [`ARCHITECTURE_CN.md`](ARCHITECTURE_CN.md) | 当前目录、职责边界、ROS 2 包和阶段 1-4 状态 |
| [`RUNTIME_PERFORMANCE_CN.md`](RUNTIME_PERFORMANCE_CN.md) | 控制链路、传感器接入、配置位置、性能方法和验收门槛 |
| [`REPORTING_CN.md`](REPORTING_CN.md) | 报告语言、批次格式、测评线、原始证据和 agent 协作规则 |

历史测试证据统一放在 [`../reports/`](../reports/)：

- [`../reports/migration/stages_1_to_3.md`](../reports/migration/stages_1_to_3.md)：阶段 1-3 迁移、构建和测试。
- [`../reports/sensors/sensor_validation.md`](../reports/sensors/sensor_validation.md)：RealSense 与 MID360 的功能和性能结论。

## 阅读顺序

1. 先看根目录 [`../README.md`](../README.md) 了解当前范围。
2. 再看架构文档确认文件归属和禁止边界。
3. 运行或接入控制/传感器前，按性能文档检查配置、QoS、话题和验收指标。

报告结论只使用“通过、未通过、条件通过、测试异常、未运行、停止”。“条件通过”表示链路
可用但仍有未满足的实时或集成门槛，不能当作整机验收通过。

最近一次无传感器机器人控制、完整控制和空场景 RTF 实测分别见
[`../reports/performance/robot_control_performance.md`](../reports/performance/robot_control_performance.md)、
[`../reports/performance/empty_scene_performance.md`](../reports/performance/empty_scene_performance.md)。
当前入口默认启动无传感器完整控制：底盘、双臂、头部和升降均启用；隔离底盘性能时才显式传入
`start_upper_body:=false`。默认 `render_hz:=30` 的完整控制已通过功能和实时验收，且可满足诊断
画面采集；`render_hz:=60` 仅用于交互式视口，不满足当前实时线。120 Hz 仿真配置与真机频率
证据也在性能报告中维护。四路 RGB-D 相机与 MID360 的三进程候选方案已达到阶段 5 入口线：
八路图像最低 `20.024 Hz`、MID360 `9.922 Hz`、控制 `/joint_states=89.998 Hz`、主 RTF
`1.637`。因此可以进入阶段 5 测试验收；这不代表正式产品化启动链路、动态外参同步和重复
稳定性已经验收完成。
