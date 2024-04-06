cmake_minimum_required(VERSION 3.8)
project(ocs2_mpc)

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
  src/LoopshapingSystemObservation.cpp
  src/mpc_base.cpp
  src/MPC_Settings.cpp
  src/SystemObservation.cpp
  src/mrt_base.cpp
  src/MPC_MRT_Interface.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  -lm -ldl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
