cmake_minimum_required(VERSION 3.8)
project(rm_drone_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rm_drone_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

include_directories(include)

foreach(plugin quadrotor_dynamics_plugin armor_impact_plugin joint_spin_plugin field_motion_plugin)
  add_library(${plugin} SHARED src/${plugin}.cpp)
  ament_target_dependencies(${plugin} rclcpp gazebo_ros gazebo_dev rm_drone_msgs std_msgs geometry_msgs)
  install(TARGETS ${plugin} LIBRARY DESTINATION lib)
endforeach()

ament_package()
