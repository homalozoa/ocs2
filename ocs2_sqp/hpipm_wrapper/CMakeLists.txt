cmake_minimum_required(VERSION 3.8)
project(hpipm_wrapper)

include_directories(include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ocs2_package()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  hpipm
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
