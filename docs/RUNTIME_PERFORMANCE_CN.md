# 运行与性能准则

## 标准入口

```bash
ros2 launch openflex_isaac_bringup sim.launch.py \
  headless:=true \
  sensor_profile:=full \
  render_hz:=30 \
  physics_hz:=120
```

主 launch 不启动 RViz。RViz 使用 `rviz_only.launch.py` 在第二个终端启动。

## 控制验收

- `controller_manager` 目标频率为 90 Hz，Isaac physics 目标为 120 Hz，两者必须分开测量。
- 不能只检查 controller 为 `active`；必须发送命令并验证底盘、升降、头部、双臂和夹爪位移。
- `/joint_states` 使用墙钟接收频率，RTF 使用同一窗口内仿真时间增量除以墙钟时间增量。
- 控制回调不得执行渲染、点云转换、图像压缩或磁盘写入。

## 传感器验收

- 四路 RGB-D 必须检查实际像素，只有 topic 频率而图像全黑不算通过。
- 深度图必须包含有限正值，RGB、depth 和 CameraInfo 的尺寸、编码与 frame 必须一致。
- VLA JPEG 合同当前目标约 15 Hz；base/left/right 为 `320x240`，head 为 `640x480`。
- MID360 性能 profile 的 `patternFiringRateHz` 为 `20000`，完整 profile 为 `36000`。
- `nearRangeM=0.1`、`farRangeM=40.0`；默认 CustomMsg 每帧最多采样 `15000` 点。
- full scan 应达到至少数万点/帧；数百点/帧属于错误配置。
- multi-tick RTX LiDAR 必须启用 `/renderer/raytracingMotion/enabled=true`。

## LiDAR 与定位限制

- `/openflex/livox_frame/lidar` 是 Isaac 原始 `PointCloud2`，`/livox/lidar_points` 是通用点云副本。
- `/livox/lidar` 是与真机驱动一致的 `livox_ros_driver2/msg/CustomMsg`。
- `/fastlio2/lio_odom` 是仿真轮式里程计兼容转发，不是 FAST-LIO 输出。
- compat bridge 已默认启用，但实际 FAST-LIO 的启动、参数、外参和轨迹精度仍需单独验收。
- 动态雷达验收必须检查时间戳、外参、运动点云和快速转动场景，静止点云通过不能替代。

## 运行检查

```bash
ros2 control list_controllers
ros2 run openflex_isaac_bringup verify_vla_runtime.py
ros2 run openflex_isaac_bringup verify_camera_images.py
ros2 topic hz /openflex/livox_frame/lidar
ros2 topic echo --once /openflex/livox_frame/lidar
```

报告保存在 `reports/runtime/full_chain/`。原始日志不进入 Git；正式结论必须记录启动参数、
Isaac Sim 版本、ROS domain、点数、频率、控制位移和已知限制。

2026-09-19 独立仓库默认参数复测：CustomMsg `6.12 Hz`、每帧 `15000` 点；通用
PointCloud2 `7.00 Hz`、中位 `69445` 点；四路 VLA JPEG 均约 `14.97 Hz`。
