cmake_minimum_required(VERSION 3.8)
project(rm_drone_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_drone_msgs REQUIRED)
find_package(rm_drone_control REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(rm_planning src/planning.cpp)
ament_target_dependencies(rm_planning rm_drone_control)
target_link_libraries(rm_planning yaml-cpp)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node rclcpp rm_drone_msgs rm_drone_control nav_msgs sensor_msgs visualization_msgs std_msgs)
target_link_libraries(mission_node rm_planning)

add_executable(referee_sim_node src/referee_sim_node.cpp)
ament_target_dependencies(referee_sim_node rclcpp rm_drone_msgs rm_drone_control nav_msgs visualization_msgs)
target_link_libraries(referee_sim_node yaml-cpp)

add_executable(plan_test src/plan_test.cpp)
target_link_libraries(plan_test rm_planning)
ament_target_dependencies(plan_test rm_drone_control)

install(TARGETS rm_planning mission_node referee_sim_node plan_test
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
