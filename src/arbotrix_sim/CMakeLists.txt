cmake_minimum_required(VERSION 3.8)
project(arbotrix_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(
  ${GAZEBO_CXX_FLAGS}
)


# =========================
# ENVIRONMENTAL SENSOR
# =========================

add_library(environmental_sensor_plugin SHARED
  src/environmental_sensor_plugin.cpp
)

ament_target_dependencies(
  environmental_sensor_plugin
  rclcpp
  std_msgs
)

target_link_libraries(
  environmental_sensor_plugin
  ${GAZEBO_LIBRARIES}
)


# =========================
# WATER PUMP
# =========================

add_library(pump_plugin SHARED
  src/pump_plugin.cpp
)

ament_target_dependencies(
  pump_plugin
  rclcpp
  std_msgs
)

target_link_libraries(
  pump_plugin
  ${GAZEBO_LIBRARIES}
)


# =========================
# INSTALL PLUGINS
# =========================

install(
  TARGETS
    environmental_sensor_plugin
    pump_plugin
  LIBRARY DESTINATION lib
)


# =========================
# INSTALL PACKAGE FILES
# =========================

install(
  DIRECTORY launch urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()