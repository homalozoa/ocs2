cmake_minimum_required(VERSION 3.8)
project(ocs2_ros_interfaces)

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ocs2_package()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/command/target_trajectories_publisher.cpp
  src/command/target_trajectories_interactive_marker.cpp
  src/command/target_trajectories_keyboard_publisher.cpp
  src/common/ros_msg_conversions.cpp
  src/common/ros_msg_helpers.cpp
  src/common/ros_logging.cpp
  src/mpc/mpc_ros_interface.cpp
  src/mrt/loopshaping_dummy_observer.cpp
  src/mrt/mrt_ros_dummy_loop.cpp
  src/mrt/mrt_ros_interface.cpp
  src/synchronized_module/ros_reference_manager.cpp
  src/synchronized_module/solver_observer_ros_callbacks.cpp
  src/visualization/visualization_helpers.cpp
  src/visualization/visualization_colors.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  -lm -ldl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  ament_auto_add_executable(test_node
    test/test_node.cpp
  )
  ament_auto_add_executable(multiplot_remap
    test/test_multiplot_remap.cpp
  )
endif()

ament_auto_package()
