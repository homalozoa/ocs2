cmake_minimum_required(VERSION 3.8)
project(ocs2_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(dependencies
  std_msgs
)

set(msg_files
  "msg/Constraint.msg"
  "msg/ControllerData.msg"
  "msg/LagrangianMetrics.msg"
  "msg/ModeSchedule.msg"
  "msg/MpcFlattenedController.msg"
  "msg/MpcInput.msg"
  "msg/MpcObservation.msg"
  "msg/MpcPerformanceIndices.msg"
  "msg/MpcState.msg"
  "msg/MpcTargetTrajectories.msg"
  "msg/Multiplier.msg"
)

set(srv_files
  "srv/Reset.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${dependencies}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
