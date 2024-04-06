cmake_minimum_required(VERSION 3.5)
project(hpipm_vendor)

find_package(ament_cmake REQUIRED)

find_package(hpipm CONFIG QUIET)

if(hpipm_FOUND)
  message("hpipm found, skip building from source")
  return()
endif()

set(EXTPRJ_NAME hpipm)
set(PKG_URL "https://github.com/giaf/hpipm")
set(PKG_VER 7c00957ffd4ef66ba5fe0fe4c1aef5654ade2999)

set(BLASFEO_PATH ${CMAKE_INSTALL_PREFIX} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)

include(ExternalProject)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  GIT_REPOSITORY ${PKG_URL}
  GIT_TAG ${PKG_VER}
  CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DBUILD_SHARED_LIBS=ON
      -DHPIPM_FIND_BLASFEO=ON
      -DHPIPM_TESTING=OFF
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  TIMEOUT 1200
)

ament_package()
