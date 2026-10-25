cmake_minimum_required(VERSION 3.8)
project(rm_ego_planner)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_drone_msgs REQUIRED)
find_package(rm_drone_control REQUIRED)
find_package(rm_drone_planner REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(rm_ego_core STATIC
  vendor/bspline_opt/src/uniform_bspline.cpp
  vendor/bspline_opt/src/bspline_optimizer.cpp
  vendor/bspline_opt/src/gradient_descent_optimizer.cpp
  vendor/path_searching/src/dyn_a_star.cpp
  vendor/traj_utils/src/polynomial_traj.cpp
  vendor/traj_utils/src/planning_visualization.cpp
  vendor/plan_manage/src/planner_manager.cpp
  vendor/plan_env/obj_predictor.cpp)
target_include_directories(rm_ego_core PUBLIC
  vendor/bspline_opt/include vendor/path_searching/include vendor/traj_utils/include
  vendor/plan_env/include vendor/plan_manage/include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(rm_ego_core rclcpp geometry_msgs visualization_msgs nav_msgs)
add_executable(ego_local_planner_node src/ego_local_planner_node.cpp)
target_include_directories(ego_local_planner_node PRIVATE include)
ament_target_dependencies(ego_local_planner_node rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs tf2_ros rm_drone_msgs rm_drone_control rm_drone_planner)
target_link_libraries(ego_local_planner_node rm_ego_core yaml-cpp)
install(TARGETS rm_ego_core ARCHIVE DESTINATION lib)
install(TARGETS ego_local_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE SOURCE.md DESTINATION share/${PROJECT_NAME})
ament_package()
