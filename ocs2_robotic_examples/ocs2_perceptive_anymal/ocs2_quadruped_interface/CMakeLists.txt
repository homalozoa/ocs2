cmake_minimum_required(VERSION 3.0)
project(ocs2_quadruped_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  ocs2_anymal_commands
  roscpp
  tf
  kdl_parser
  robot_state_publisher
  segmented_planes_terrain_model
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/QuadrupedDummyNode.cpp
  src/QuadrupedInterface.cpp
  src/QuadrupedLogger.cpp
  src/QuadrupedMpc.cpp
  src/QuadrupedMpcNode.cpp
  src/QuadrupedPointfootInterface.cpp
  src/QuadrupedTfPublisher.cpp
  src/QuadrupedVisualizer.cpp
  src/SwingPlanningVisualizer.cpp
  src/TerrainPlaneVisualizer.cpp
  src/TerrainReceiver.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
