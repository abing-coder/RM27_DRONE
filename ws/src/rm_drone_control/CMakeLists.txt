cmake_minimum_required(VERSION 3.8)
project(rm_drone_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_drone_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(px4_msgs QUIET)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(fc_sim_node src/fc_sim_node.cpp)
ament_target_dependencies(fc_sim_node rclcpp rm_drone_msgs nav_msgs sensor_msgs std_msgs)

add_executable(position_controller_node src/position_controller_node.cpp)
ament_target_dependencies(position_controller_node rclcpp rm_drone_msgs nav_msgs std_msgs)

install(TARGETS fc_sim_node position_controller_node DESTINATION lib/${PROJECT_NAME})

if(px4_msgs_FOUND)
  add_executable(px4_bridge_node src/px4_bridge_node.cpp)
  ament_target_dependencies(px4_bridge_node rclcpp rm_drone_msgs nav_msgs px4_msgs)
  install(TARGETS px4_bridge_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "px4_msgs 未找到，跳过 px4_bridge_node（实机阶段再把 px4_msgs 放进工作空间）")
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
