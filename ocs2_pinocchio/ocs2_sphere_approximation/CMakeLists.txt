cmake_minimum_required(VERSION 3.8)
project(ocs2_sphere_approximation)

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ocs2_package()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/SphereApproximation.cpp
  src/PinocchioSphereInterface.cpp
  src/PinocchioSphereKinematics.cpp
  src/PinocchioSphereKinematicsCppAd.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
