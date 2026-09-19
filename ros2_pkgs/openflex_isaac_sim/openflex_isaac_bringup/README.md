# OpenFleX Isaac Sim ROS 2 Bringup

This package launches the OpenFleX robot, ros2_control, four RGB-D cameras,
the Livox MID360 point cloud, and the Livox IMU in Isaac Sim. The main
simulation launch never starts RViz; RViz is a separate launch entry.

The documented and validated path is **Isaac Sim 6.0** with ROS 2 Humble.
The older `isaac_sim.launch.py` entry point is retained for compatibility, but
the complete Isaac Sim 6.0 workflow uses `sim.launch.py` below.

## Prerequisites

- Ubuntu with ROS 2 Humble sourced.
- Isaac Sim 6.0 installed at `/home/1024201092WYH/isaacsim-6.0`, or pass a
  different installation with `isaac_path:=...`.
- A graphical session when using `headless:=false`; set `DISPLAY` to the
  active display.
- The workspace built and sourced before launching.

## Build

```bash
cd /home/1024201092WYH/openflex_all/isaacsim_robot
source /opt/ros/humble/setup.bash
source /home/1024201092WYH/openflex_all/openflex_ws/install/setup.bash
colcon build --symlink-install \
  --base-paths ros2_pkgs/openflex_isaac_sim .deps/src \
  --packages-up-to openflex_isaac_bringup
source install/setup.bash
```

## Isaac Sim 6.0 full-chain launch

The following is the known-good interactive command used for the full-chain
validation. `sensor_profile:=full` enables all four RGB-D cameras, MID360,
and IMU. `lidar_transport:=helper` is the recommended stable MID360 path for
Isaac Sim 6.0.

With no launch arguments, `sim.launch.py` defaults to `headless:=true`,
`render_hz:=30.0`, `physics_hz:=120.0`, `sensor_profile:=full`,
`lidar_transport:=helper`, `lidar_object_id_map:=false`, and
`start_upper_body:=true`, `show_lift_mast:=true`, and `ros_domain_id:=49`.
It does not start RViz.

```bash
cd /home/1024201092WYH/openflex_all/isaacsim_robot
source /opt/ros/humble/setup.bash
source /home/1024201092WYH/openflex_all/openflex_ws/install/setup.bash
source install/setup.bash

export ISAACSIM_ROBOT_ROOT="$PWD"
export DISPLAY=:0
export ROS_DOMAIN_ID=49
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

ros2 launch openflex_isaac_bringup sim.launch.py \
  headless:=false \
  render_hz:=30 \
  physics_hz:=120 \
  sensor_profile:=full \
  lidar_transport:=helper \
  lidar_object_id_map:=false \
  start_upper_body:=true \
  isaac_path:=/home/1024201092WYH/isaacsim-6.0 \
  api_port:=8085
```

For a server or CI machine without a display, use `headless:=true` and omit
`DISPLAY`. Keep `ROS_DOMAIN_ID` and the RMW settings identical in every ROS 2
terminal that must communicate with the simulation. With
`ROS_LOCALHOST_ONLY=1`, remote machines cannot discover this ROS graph.

Launch parameters and defaults:

- `headless`: defaults to `true`; use `false` for the Isaac Sim GUI.
- `render_hz`: defaults to `30.0`.
- `physics_hz`: defaults to `120.0`.
- `sensor_profile`: defaults to `full`; choices are `none`, `minimal`, `lidar`, and `full`.
- `lidar_transport`: defaults to `helper`, the stable Isaac Sim 6.0 MID360 path.
- `lidar_mount_mode`: defaults to `parented`, which keeps the MID360 below the robot USD mount so it follows base motion. `fixed_kinematic` and `fixed` are diagnostic modes.
- `lidar_object_id_map`: defaults to `false` for the current MID360 transport.
- `start_upper_body`: defaults to `true` and loads arm, head, and lift controllers.
- `show_head_camera`: defaults to `true`; set it to `false` only while debugging the physical head-camera mesh.
- `show_lift_mast`: defaults to `true`; set it to `false` only to hide the mast visual while retaining collision and lift control.
- `ros_domain_id`: defaults to `49` for both simulation and standalone RViz launches.
- `isaac_path`: auto-detects Isaac Sim and currently resolves to `/home/1024201092WYH/isaacsim-6.0`.
- `api_host` and `api_port`: default to `127.0.0.1` and `8085`. Launch fails fast
  when the port is already occupied, preventing a new launch from attaching to a stale Isaac process.
- `stage`: defaults to `auto` and uses the package's empty validation stage.
- `use_sim_time`: defaults to `true`; `controller_use_sim_time` defaults to `false`.
- `spawn_wait_timeout` and `sim_ready_timeout`: default to `900.0` and `180.0` seconds.
- `x/y/z`: default to `0.0/0.0/0.25` metres.
- `roll/pitch/yaw`: default to `0.0/0.0/0.0` radians.
- `fixed`: defaults to `false`.

## Launch profiles

`sim.launch.py` supports these sensor profiles:

- `none`: robot control only.
- `minimal`: four RGB-D cameras.
- `lidar`: MID360 point cloud and IMU.
- `full`: four RGB-D cameras, MID360 point cloud, and IMU.

Useful options include:

```bash
# Control-only smoke test
ros2 launch openflex_isaac_bringup sim.launch.py sensor_profile:=none

# Full sensors
ros2 launch openflex_isaac_bringup sim.launch.py \
  sensor_profile:=full

# Use another Isaac Sim installation
ros2 launch openflex_isaac_bringup sim.launch.py \
  isaac_path:=/path/to/isaacsim-6.0
```

The `sim.launch.py` interface has no `use_rviz` argument. To start RViz after
the simulation is running, use a second terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/1024201092WYH/openflex_all/isaacsim_robot/install/setup.bash
ros2 launch openflex_isaac_bringup rviz_only.launch.py use_sim_time:=true
```

The standalone RViz launch defaults to `ros_domain_id:=49`,
`ros_localhost_only:=1`,
`simulation_mode:=true`, `joint_states_topic:=/joint_states`,
`position_command_topic:=/lift_position_controller/commands`, and simulated
lift limits `pos_min:=-0.65` / `pos_max:=0.3`. In simulation mode the lift
panel reads `lift_joint` from `/joint_states` and directly commands the active
Isaac lift position controller. Hardware RViz entries must leave
`simulation_mode` disabled.

The control RViz profile starts with MID360 `/livox/lidar_points`, projected `/scan`,
`/odom`, and lightweight axes for `base_link` and `livox_frame` so the mount
offset remains visible without drawing the complete robot TF tree.
The real serial battery overlay remains disabled. Display/base/lift panels use
left-side tabs and head/arm panels use right-side tabs so the 3D view remains
usable on a 1920x1080 display. The Orbit view matches the validated 5.1 RViz profile.

The interactive Isaac viewport preserves the 5.1 viewing direction and adapts
it to the Isaac Sim 6.0 metre-scale robot: `eye=[2.0, 2.0, 1.5]` and
`target=[0.0, 0.0, 0.85]`. Using the raw 5.1 coordinates clips the head and base in 6.0. The real
`lift_link.STL` mast remains part of collision and control and is visible by
default; pass `show_lift_mast:=false` only when temporarily diagnosing visual
occlusion.

## Controllers and command topics

When `start_upper_body:=true`, the following controllers must become active:

- `joint_state_broadcaster`
- `swerve_drive_controller`
- `left_forward_position_controller`
- `right_forward_position_controller`
- `head_forward_position_controller`
- `lift_position_controller`

Commands use these topics:

- `/cmd_vel` (`geometry_msgs/msg/Twist`) — mobile base velocity.
- `/left_forward_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`) — left arm joints.
- `/right_forward_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`) — right arm joints.
- `/head_forward_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`) — head yaw and pitch.
- `/lift_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`) — lift joint.
- `/velocity_controller/commands`
  (`std_msgs/msg/Float64MultiArray`) — VLA lift velocity action. In simulation,
  `openflex_vla_contract_bridge` safely integrates this command into
  `/lift_position_controller/commands` with position limits and a command timeout.

The Isaac/ros2_control bridge uses:

- `/openflex/joint_states` — simulation joint state input.
- `/openflex/joint_command` — aligned joint command output.
- `/joint_states` — public joint state stream.
- `/odom` — mobile base odometry.
- `/clock` — simulation clock.

## Sensor topics

Each camera publishes `640x480` data under its namespace:

- `/cam_base/{color,depth}/image`
- `/cam_head/{color,depth}/image`
- `/cam_left/{color,depth}/image`
- `/cam_right/{color,depth}/image`
- `/cam_{base,head,left,right}/color/camera_info`

Isaac Sim publishes directly to the released `/cam_*` contract. There is no
relay and no second camera-image namespace. The four cameras
therefore produce eight image topics in total: four RGB and four depth images.

The validated message encodings are `rgb8` for color and `32FC1` for depth.
The VLA compressed-image contract is separately rate-limited to 15 Hz and uses
the real data schema sizes: base/left/right are `320x240`, while head is
`640x480`.
The MID360 and IMU topics are:

- `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`), the real-driver and FAST-LIO input contract.
- `/livox/lidar_points` (`sensor_msgs/msg/PointCloud2`), the RViz and generic point-cloud contract.
- `/scan` (`sensor_msgs/msg/LaserScan`), projected from the simulated MID360 cloud.
- `/openflex/livox_frame/lidar` (`sensor_msgs/msg/PointCloud2`), frame
  `livox_frame`; this is the raw Isaac Sim stream and must contain non-empty points and Livox metadata.
- `/livox/imu` (`sensor_msgs/msg/Imu`), frame `livox_frame`.

## Runtime checks

After the launch settles, check controller state and representative streams:

```bash
ros2 control list_controllers
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /cam_base/color/image
ros2 topic hz /cam_base/color/image/compressed
ros2 topic hz /fastlio2/lio_odom
ros2 run openflex_isaac_bringup verify_vla_runtime.py
ros2 topic echo --once /cam_base/depth/camera_info
ros2 run openflex_isaac_bringup verify_camera_images.py
ros2 topic hz /openflex/livox_frame/lidar
ros2 topic hz /livox/imu
ros2 topic echo --once /openflex/livox_frame/lidar
```

A successful full-chain run must show all six controllers active, continuous
clock/joint/odometry traffic, four valid RGB-D camera contracts, a non-empty
MID360 raw and compatibility streams, and a live `/livox/imu`. Control validation must include
base displacement plus movement of the lift, head, left arm, and right arm.
Camera validation must inspect pixel values: all-black RGB or depth with no
finite positive samples is a failure even when ROS messages are arriving.

For VLA, `/fastlio2/lio_odom` has the same name and
`nav_msgs/msg/Odometry` type as the real robot, but its simulation source is a
relay of `/odom`; it is not the output of FAST-LIO. The simulation
`/livox/lidar` now matches the real driver's `livox_ros_driver2/msg/CustomMsg`
type, but FAST-LIO parameters, extrinsics, timing, and trajectory accuracy still
require a separate algorithm-level validation.

The latest validated report is:

`reports/runtime/full_chain/isaac6_full_chain_20260919_8dof.json`

That run passed controller, control, sensor, and infrastructure checks. It
measured approximately `0.698997 m` base displacement, validated all eight
RGB-D image topics with non-black/finite-positive pixel checks, observed
`50,043` MID360 points, and exercised live dual-arm,
gripper, head, lift, and base control before restoring the initial pose. The
base camera uses the real-robot mount translation `[0.36, 0, 0.055] m` and
the Isaac Sim USD Camera quaternion `[0.5, 0.5, -0.5, -0.5]`. This maps local
`-Z` to robot `+X` and local `+Y` to robot `+Z`, so the camera looks forward
without the previous 90-degree image roll. RGB and depth use the real-camera
frame names `d435_color_optical_frame` and `d435_depth_optical_frame`.
Each `/cam_*` contract publishes color image, depth image, color CameraInfo,
depth CameraInfo, and a subscriber-driven JPEG compressed color topic capped at
15 Hz. These are the normalized OpenFleX real-robot application topics; the
simulation intentionally does not duplicate the RealSense driver's lower-level
`/camera/d435/...` aliases. The corrected screenshot and camera contract report
are stored under `reports/runtime/camera_validation/20260919_1135_base_upright/`
and `reports/runtime/camera_validation/20260919_1140_camera_contract/`.
The head and wrist camera poses are coordinate-equivalent to the 5.1 USD
transforms. Wrist depth is metric `32FC1`; RViz must use Best
Effort QoS and metre-scale display ranges. No Isaac Sim fatal error was observed.

The current simulated rigid-body mass is approximately `60.8993 kg`. This is
not a physical robot weight: the generated URDF currently assigns placeholder
`0.001 kg` inertials to each of the three head links. Replace those values only
when authoritative CAD/BOM mass and inertia data are available. The head-link
visual transforms close consistently with the existing OpenFleX description;
the vertical member visible behind the head is the lift mast, not a head mesh.

## Shutdown and troubleshooting

Stop the launch with `Ctrl-C`. If a previous run was interrupted, stop stale
Isaac Sim and ROS 2 processes before retrying. For Isaac Sim 6.0, keep these
settings unless diagnosing a specific issue:

- `lidar_transport:=helper`
- `lidar_object_id_map:=false`
- `render_hz:=30`
- `physics_hz:=120`

The repository also contains the legacy `isaac_sim.launch.py`; do not use it
as the Isaac Sim 6.0 full-chain acceptance command.
