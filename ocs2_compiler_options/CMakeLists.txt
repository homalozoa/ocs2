cmake_minimum_required(VERSION 3.8)
project(ocs2_compiler_options)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "ocs2_utils-extra.cmake")
