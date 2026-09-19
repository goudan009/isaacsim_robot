cmake_minimum_required(VERSION 3.8)
project(openflex_isaac_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
# Install rviz configs
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/camera_contract_publisher.py
    scripts/lidar_contract_publisher.py
    scripts/vla_contract_bridge.py
    scripts/robot_description_publisher.py
    scripts/isaacsim_compat_bridge.py
    scripts/verify_camera_images.py
    scripts/start_robot_control_sim.py
    scripts/setup_loaded_robot_when_ready.py
    scripts/spawn_robot_when_ready.py
    scripts/wait_for_sim_ready.py
    scripts/verify_mobile_base_runtime.py
    scripts/verify_upper_body_runtime.py
    scripts/verify_vla_runtime.py
    scripts/run_remote_rviz_client.sh
    scripts/stop_stale_vr_processes.py
    scripts/isaacsim_vr_arm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
