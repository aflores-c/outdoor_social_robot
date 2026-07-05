cmake_minimum_required(VERSION 3.8)
project(ndt_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── ndt_map_creator ──────────────────────────────────────────────────────────
add_executable(ndt_map_creator src/ndt_map_creator.cpp)
ament_target_dependencies(ndt_map_creator rclcpp pcl_conversions pcl_ros)
target_link_libraries(ndt_map_creator ${PCL_LIBRARIES})

# ── ndt_localizer ────────────────────────────────────────────────────────────
add_executable(ndt_localizer src/ndt_localizer.cpp)
ament_target_dependencies(ndt_localizer
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  pcl_conversions pcl_ros)
target_link_libraries(ndt_localizer ${PCL_LIBRARIES})

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS ndt_map_creator ndt_localizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
