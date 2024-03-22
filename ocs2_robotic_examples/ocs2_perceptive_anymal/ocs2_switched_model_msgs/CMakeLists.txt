cmake_minimum_required(VERSION 3.8)
project(ocs2_switched_model_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(dependencies
  std_msgs
  ocs2_msgs
)

set(msg_files
  "msg/Gait.msg"
  "msg/GaitSequence.msg"
  "msg/ScheduledGaitSequence.msg"
)

set(srv_files
  "srv/TrajectoryRequest.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${dependencies}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
