cmake_minimum_required(VERSION 3.8)
project(isaacsim_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_isaac_urdf.py
    scripts/inspect_isaac_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
