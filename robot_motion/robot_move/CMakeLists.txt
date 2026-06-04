cmake_minimum_required(VERSION 3.8)
project(robot_move)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(base_navigation REQUIRED)
find_package(play_motion2_msgs REQUIRED)

# ── Shared library ──────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/robot_move.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  base_navigation
  play_motion2_msgs
)

# ── Install ─────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Export for downstream packages ──────────────────────────────────────────
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action base_navigation play_motion2_msgs)

ament_package()
