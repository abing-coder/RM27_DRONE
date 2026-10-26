cmake_minimum_required(VERSION 3.8)
project(rm_drone_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_acceptance.py DESTINATION lib/${PROJECT_NAME})

ament_package()
