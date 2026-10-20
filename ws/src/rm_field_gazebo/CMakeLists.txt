cmake_minimum_required(VERSION 3.8)
project(rm_field_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gen_field.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/asset_meshes.py DESTINATION lib/${PROJECT_NAME})

ament_package()
