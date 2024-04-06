cmake_minimum_required(VERSION 3.5)
project(blasfeo_vendor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)

set(EXTPRJ_NAME blasfeo)
set(PREFIX_DIR "${CMAKE_CURRENT_BINARY_DIR}/${EXTPRJ_NAME}")
set(OUT_DIR "${CMAKE_BINARY_DIR}/${EXTPRJ_NAME}")
set(PKG_URL "https://github.com/giaf/blasfeo")
set(PKG_VER b39df4d6774dcf37aa759ea65a26f2c8affff281)

include(ExternalProject)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  GIT_REPOSITORY ${PKG_URL}
  GIT_TAG ${PKG_VER}
  CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DBLASFEO_EXAMPLES=OFF
      -DBUILD_SHARED_LIBS=ON
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  TIMEOUT 1200
)

ament_package()
