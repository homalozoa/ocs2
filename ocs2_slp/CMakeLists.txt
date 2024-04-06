cmake_minimum_required(VERSION 3.8)
project(ocs2_slp)

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ocs2_package()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/pipg/PipgSettings.cpp
  src/pipg/PipgSolver.cpp
  src/pipg/SingleThreadPipg.cpp
  src/Helpers.cpp
  src/SlpSettings.cpp
  src/SlpSolver.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  -lm -ldl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
