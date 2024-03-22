macro(ocs2_package)
  # Default to C99
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
  endif()

    execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
    message(STATUS "Architecture: ${ARCHITECTURE}")

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        # the line below will cause a lot of errors...
        # -Wall -Wextra -Wpedantic
        -Wfatal-errors
        -Wl,--no-as-needed
        -fomit-frame-pointer -g0 -finline-functions -floop-unroll-and-jam
        -O2 -Werror
        -DBOOST_ALL_DYN_LINK
    )
  endif()
  # might not be necessary
  # if (${ARCHITECTURE} STREQUAL "aarch64" OR META_BUILDING STREQUAL "ON")
  #   add_compile_options(-march=armv8-a)
  # elseif (${ARCHITECTURE} STREQUAL "x86_64")
  #   add_compile_options(-march=native)
  # endif()
endmacro()
