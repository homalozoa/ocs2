cmake_minimum_required(VERSION 3.8)
project(ocs2_self_collision_visualization)

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ocs2_package()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/GeometryInterfaceVisualization.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
