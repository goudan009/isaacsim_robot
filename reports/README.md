# 测试报告总览

所有报告遵循 [`../docs/REPORTING_CN.md`](../docs/REPORTING_CN.md)。报告正文统一使用中文；
代码、配置键、包名和 ROS 2 话题保留原始标识。

## 当前总览

| 类别 | 最新有效批次 | 功能结论 | 实时结论 | 固定报告 |
| --- | --- | --- | --- | --- |
| 阶段 1-3 迁移 | 2026-09-06 | 通过 | 不适用 | [`migration/stages_1_to_3.md`](migration/stages_1_to_3.md) |
| 无传感器机器人控制 | `20260909-144150-完整控制-codex` | 通过 | 通过 | [`performance/robot_control_performance.md`](performance/robot_control_performance.md) |
| 空场景性能 | 20260909-102402-空场景-codex | 不适用 | 通过 | [`performance/empty_scene_performance.md`](performance/empty_scene_performance.md) |
| 独立传感器 | `20260909-160015-独立相机-quad-codex` | 通过 | 条件通过 | [`sensors/sensor_validation.md`](sensors/sensor_validation.md) |
| 三进程整机相机 + MID360 实验 | `20260911-233603-双进程整机四相机-phase5-candidate-cpu64-native-no-object-map` | 通过 | 通过 | 八路图像 `20.024–20.622 Hz`；MID360 `9.922 Hz`、非空 `198`；联合线为 `8 Hz`；阶段 5 可进入；[`sensors/sensor_validation.md`](sensors/sensor_validation.md) |
| 控制与传感器产品化组合 | 阶段 5 入口批次已完成 | 通过 | 待验收 | 正式启动链路、动态外参和重复稳定性仍需阶段 5 验收 |

## 固定报告与原始证据

| 类别 | 结论报告 | 原始证据 |
| --- | --- | --- |
| 迁移 | `migration/stages_1_to_3.md` | 测试框架输出或对应批次目录 |
| 机器人控制 | `performance/robot_control_performance.md` | `performance/raw/robot_control_baseline/`、`performance/raw/robot_control_full/` |
| 空场景 | `performance/empty_scene_performance.md` | `performance/raw/empty_scene/` |
| 传感器 | `sensors/sensor_validation.md` | `sensors/raw/` |

原始 JSON、启动日志和截图用于复核，不能直接替代结论。后续 agent 必须新建独立批次目录，
不得覆盖其他批次的证据，也不得新建日期型结论报告。

## Git 提交与本地归档边界

提交到 `main` 的内容必须满足“可复现、可审阅、与机器无关”：

- 提交：仿真源码、ROS 2 包、配置、必要的 USD/机器人资产、测试脚本、固定结论报告和文档。
- 不提交：`build/`、`install/`、`log/`、Python 缓存、`reports/**/raw/`、
  `reports/runtime/generated/` 以及包含本机绝对路径的生成 URDF。
- 本地归档：原始 `result.json`、启动日志、CSV、截图和录包保留在对应 `reports/**/raw/` 批次目录，
  用于复核，不作为 GitHub 主线源码。需要对外发布时，应单独打包并记录校验值。
- 若某个原始证据必须长期共享，先将它压缩到仓库外的归档存储，再在固定结论报告中记录归档位置和校验值。

当前首个 `main` 提交的目标是“阶段 1-5 入口代码与结论”，不包含本机生成物和历史调试日志。
