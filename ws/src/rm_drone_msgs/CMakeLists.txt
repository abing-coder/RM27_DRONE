cmake_minimum_required(VERSION 3.8)
project(rm_drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttitudeTarget.msg"
  "msg/MotorCommand.msg"
  "msg/TrajectorySetpoint.msg"
  "msg/ArmorDetection.msg"
  "msg/ArmorDetectionArray.msg"
  "msg/ImpactEvent.msg"
  "msg/RefereeStatus.msg"
  "msg/MissionState.msg"
  "msg/EstimatorStatus.msg"
  "msg/PlannerGoal.msg"
  "msg/LocalTrajectory.msg"
  "msg/PlannerStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
