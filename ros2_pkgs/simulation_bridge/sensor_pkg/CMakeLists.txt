cmake_minimum_required(VERSION 3.8)
project(isaacsim_sensors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../isaac_sim_core/assets
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../isaac_sim_core/config/sensor_params/mid360
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/realsense_standalone.py
    scripts/realsense_smoke.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../isaac_sim_core/components/sensors/mid360/mid360_semantic_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
