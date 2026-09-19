# 阶段 2 迁移报告：openflex_isaac_sim

日期：2026-09-19
范围：旧分包布局 → `isaacsim_robot/ros2_pkgs/openflex_isaac_sim`

## 结果

旧 `isaacsim_*` ROS 2 包内容已迁入独立 `isaacsim_robot` 仓库下的
`openflex_isaac_*` 包族。仿真源码不再放入 OpenFleX 主工作空间，colcon 可识别 6 个包。

| 旧包 | 新包 | 状态 |
| --- | --- | --- |
| isaacsim_bringup | openflex_isaac_bringup | 已补齐 launch（rviz_only、vr_teleop）、脚本（compat_bridge、vr_arm_node、setup_loaded_robot、verify_mobile_base/upper_body、stop_stale_vr、run_remote_rviz_client）、controllers.isaac.upper_body.yaml、remote_standard.rviz、robot_control_only.rviz |
| isaacsim_command_controller | openflex_isaac_controllers | 已迁移（本次未改动） |
| isaacsim_description | openflex_isaac_description | 已迁移（本次未改动） |
| isaacsim_embodiment_contract | openflex_isaac_contract | 新建；包名引用已更新；verify 脚本传感器 YAML 回退路径适配 Robot/isaacsim_robot 布局 |
| isaacsim_sensors | openflex_isaac_sensors | 补齐 config/calibration/ 与 scripts/（realsense_smoke、realsense_standalone）；setup.py 安装校准配置和脚本 |
| （新增） | openflex_isaac_bridge | 桥接层（本次未改动） |

## 验证证据

| 检查 | 结果 |
| --- | --- |
| `colcon list` 识别 openflex_isaac_* | 6 个包 |
| `colcon build --packages-select openflex_isaac_sensors openflex_isaac_contract openflex_isaac_bringup` | 3 包通过（sensors 仅 pkg_resources deprecation 警告） |
| `verify_embodiment_contract.py`（新包名） | `[OK] OpenFleX embodiment contract matches Isaac controller and sensor configs` |
| 全部 launch 文件语法编译 | 5 个通过 |

## 后续修订

1. 2026-09-19 已删除旧 `ros2_pkgs/control` 和 `ros2_pkgs/simulation_bridge` 跟踪内容。
2. 已修复 standalone 路径解析、MID360 发射率和 multi-tick Motion BVH 启动设置。
3. 已补充 VLA 接口、控制和 RGB-D 报告；`/fastlio2/lio_odom` 仍明确标记为 `/odom` 兼容转发。
4. 默认 `/livox/lidar` 已改为真机兼容的 `CustomMsg`，`/livox/lidar_points` 保留通用
   `PointCloud2`；FAST-LIO 算法本身及其外参、时间同步和轨迹精度仍需单独验收。
