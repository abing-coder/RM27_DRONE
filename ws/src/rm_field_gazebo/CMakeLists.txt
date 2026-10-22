cmake_minimum_required(VERSION 3.8)
project(rm_field_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gen_field.py scripts/gen_official_field.py scripts/step_xde.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/asset_meshes.py scripts/field_appearance.py scripts/step_mesh.py scripts/requirements-cad.txt DESTINATION lib/${PROJECT_NAME})

ament_package()
