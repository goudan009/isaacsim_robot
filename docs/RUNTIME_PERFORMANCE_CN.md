# 运行与性能准则

本文只说明测试方法、指标定义和接入原则；具体实测结果统一查看：

- [`../reports/performance/robot_control_performance.md`](../reports/performance/robot_control_performance.md)
- [`../reports/performance/empty_scene_performance.md`](../reports/performance/empty_scene_performance.md)
- [`../reports/sensors/sensor_validation.md`](../reports/sensors/sensor_validation.md)

## 机器人控制

入口：

```bash
ros2 launch isaacsim_bringup robot_control_only.launch.py \
  headless:=true rviz:=false
```

当前配置目标是 physics `120 Hz`、controller manager `90 Hz`。这三个指标必须分开测量：

- `/joint_states`：ROS 订阅回调按墙钟测得的实际接收频率。
- `/openflex/joint_states`：Isaac physics graph 的仿真状态，用于估计 physics step rate。
- `RTF`：同一有效窗口内的仿真时间增量除以墙钟时间增量。

控制验收不能只看 controller 是否为 `active`，还必须验证底盘、升降、头部、左臂和右臂的
实际 command-to-state 位移响应。测试脚本位于 `test/performance/`。

无传感器完整控制默认 `render_hz:=30`，可用于诊断画面采集。实测该配置的 physics step rate 为
`175.689 Hz`、RTF 为 `1.464`。将渲染频率提高到 `60 Hz` 时，physics step rate 降到 `103.032 Hz`、
RTF 降到 `0.859`；因此 `60 Hz` 仅适合需要交互式 Isaac 视口的场景，不是当前实时控制配置。

显式使用 `camera_profile:=quad` 时，会在已导入机器人上挂载四路 `640×480 @ 30 Hz` RGB-D 相机，
不启动雷达、SLAM 或旧全身组合入口。当前一次有效负载探针中，physics step rate 仅 `7.310 Hz`、
RTF `0.061`，各 RGB/深度流仅约 `1.68–1.78 Hz` 墙钟频率，因此该组合未通过，不能用于控制或采集。

历史双进程思路已在完整机器人上做过一次只读状态副本实验：物理权威进程无相机并保持
`133.795 Hz`、RTF `1.115`，副本进程接收关节和里程计快照后渲染四台 RGB-D 相机。隔离保护了
主控制实时性，但副本八路 ROS 图像实际只有 `13.195–13.941 Hz`，仍未达到每路 `29 Hz` 的采集线。
该实验不是阶段 4 产品化组合，完整证据和限制见
[`../reports/sensors/sensor_validation.md`](../reports/sensors/sensor_validation.md)。

后续参数扫描表明，低频副本的相机 tick 会导致漏采样。当前最低可采集配置为：副本
physics/render `55/30 Hz`、四台 `640×480` RGB-D 相机采样 tick `40 Hz`、ROS 2 发布队列线程开启。
2026-09-09 的 30 秒测量中，八路图像为 `22.126–23.162 Hz`，主控制仍为 physics `126.361 Hz`、
RTF `1.053`、`/joint_states 90.009 Hz`。这满足每路 `20 Hz` 的最低采集线，但仍未满足 `29 Hz`
正式目标，不能宣称为 `30 Hz` 相机方案或阶段 4 验收通过。

当前三进程联合方案将 MID360 从视觉副本中隔离为独立雷达进程：控制权威进程负责机器人和
`/clock`，相机进程负责四路 RGB-D，雷达进程使用历史 `mid360_empty_stage.usda` 和
`MID360_PERFORMANCE`。最新有效批次（`20260911-203737-双进程整机四相机-robot-camera-lidar-physics30-20260911`）中，
主进程 `/joint_states=90.001 Hz`、physics=`193.817 Hz`、RTF=`1.615`；八路图像为
`20.308–21.503 Hz`；MID360 原始接收为 `9.822 Hz`，非空 `295`，末帧 `59080` 点。
因此联合整机的当前最低可用线为：相机每路不低于 `20 Hz`，MID360 非空 PointCloud2
墙钟频率不低于 `8 Hz`。最新阶段 5 入口批次已达到相机 `20.024–20.622 Hz`、MID360
`9.922 Hz`；独立 MID360 历史性能线仍为 `10 Hz`，不能混用。

MID360 经权威时钟重打包时，`elapsed_wall_s` 从进程启动开始计算，可能包含等待首条消息的
启动阶段；报告不得直接用总运行窗口吞吐代替传感器有效频率。应优先使用
`input_active_wall_hz`、`output_active_wall_hz`，并同时记录首条消息启动延迟和有效窗口时长。

## 控制性能原则

- 控制回调不执行渲染、点云转换、JSON 序列化或磁盘写入。
- 物理进程和传感器进程需要隔离时，必须额外验证状态快照、command/state 延迟和控制 gap。
- 120 Hz physics 的单步预算是 `8.333 ms`；平均频率和 P99/max gap 都要记录。
- `controller_manager.update_rate=90` 是调度目标，不代表所有负载下都能保证 90 Hz。
- 真机频率必须在真机进程运行时单独测量：

  ```bash
  ros2 topic hz /joint_states --window 100
  ```

## 传感器接入

传感器 canonical 配置位于 `isaac_sim_core/config/sensor_params/`，ROS 2 安装副本位于
`ros2_pkgs/simulation_bridge/sensor_pkg/config/`，两者必须同步。

RealSense 和 MID360 应通过 parent/mount/local pose 接入，不能把传感器渲染和写盘逻辑塞进
控制回调。每个物理相机只创建一个 RenderProduct；图像和点云使用有界 latest-wins 队列，
并记录 source/output drops。

nominal 标定只用于仿真接口检查，实机接入前必须替换真实内参、畸变、外参和 depth scale。
MID360 的 raw 频率不能直接当作 Python CustomMsg bridge 频率；两者分别测量和记录。

## 测试与报告规则

- 测试代码只放 `test/`，结论只放 `reports/`。
- 同一类测试使用固定 Markdown 文件，按批次表格追加，不按每次运行创建日期文档。
- JSON、启动日志等原始证据按批次放在 `reports/<类别>/raw/<测试项>/<批次编号>/`。
- 报告结论只使用“通过、未通过、条件通过、测试异常、未运行、停止”；程序内部状态码仅保留在原始 JSON。
- 具体批次格式、测评线和 agent 写入流程以 [`REPORTING_CN.md`](REPORTING_CN.md) 为准。
- 阶段 5 入口性能线已通过；正式产品化仍需完成启动链路、动态外参、时间一致性和重复稳定性验收。
