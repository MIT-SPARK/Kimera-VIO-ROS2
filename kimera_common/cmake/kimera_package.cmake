#
# Standard Kimera project setup
#
# @public
#
macro(kimera_package)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -Wall
      -Wextra
      -Wpedantic
      -Werror
      -Wdeprecated
    )
  endif()
endmacro()
