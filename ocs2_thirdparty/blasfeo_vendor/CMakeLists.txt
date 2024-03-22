cmake_minimum_required(VERSION 3.5)
project(blasfeo_vendor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)

set(EXTPRJ_NAME blasfeo)
set(PREFIX_DIR "${CMAKE_CURRENT_BINARY_DIR}/${EXTPRJ_NAME}")
set(OUT_DIR "${CMAKE_BINARY_DIR}/${EXTPRJ_NAME}")
set(PKG_URL "https://github.com/giaf/blasfeo")
set(PKG_VER ae6e2d1dea015862a09990b95905038a756ffc7d)

include(ExternalProject)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  GIT_REPOSITORY ${PKG_URL}
  GIT_TAG ${PKG_VER}
  CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${PREFIX_DIR}
      -DBLASFEO_EXAMPLES=OFF
      -DBUILD_SHARED_LIBS=ON
  INSTALL_DIR "${PREFIX_DIR}"
  TIMEOUT 1200
)

set(blasfeo
  blasfeo
)

install(DIRECTORY ${OUT_DIR}/include
  DESTINATION include/${EXTPRJ_NAME}
)

install(DIRECTORY ${OUT_DIR}/lib/
  DESTINATION lib/
  REGEX ".?so.?"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    GROUP_EXECUTE GROUP_READ
)

ament_export_include_directories(include)
ament_export_libraries(blasfeo)
ament_package()
