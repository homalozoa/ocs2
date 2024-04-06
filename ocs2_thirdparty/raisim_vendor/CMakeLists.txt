cmake_minimum_required(VERSION 3.5)
project(raisim_vendor)

find_package(ament_cmake REQUIRED)

find_package(raisim CONFIG QUIET)

if(raisim_FOUND)
  message("raisim found, skip building from source")
  return()
endif()

set(EXTPRJ_NAME raisim)
set(GIT_URL "https://github.com/raisimTech/raisimLib.git")
set(PKG_VER v1.1.7)

include(ExternalProject)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  GIT_REPOSITORY ${GIT_URL}
  GIT_TAG ${PKG_VER}
  CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DRAISIM_EXAMPLE=OFF
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  TIMEOUT 1200
)

ament_package()
