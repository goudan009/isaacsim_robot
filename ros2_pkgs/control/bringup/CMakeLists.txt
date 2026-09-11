cmake_minimum_required(VERSION 3.8)
project(isaacsim_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ../../../isaac_sim_core/config/sensor_params/sensors.isaac.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY ../../../isaac_sim_core/assets
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/isaacsim_compat_bridge.py
    scripts/start_robot_control_sim.py
    scripts/setup_loaded_robot_when_ready.py
    scripts/spawn_robot_when_ready.py
    scripts/wait_for_sim_ready.py
    scripts/verify_mobile_base_runtime.py
    scripts/verify_upper_body_runtime.py
    scripts/run_remote_rviz_client.sh
    scripts/stop_stale_vr_processes.py
    scripts/isaacsim_vr_arm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
