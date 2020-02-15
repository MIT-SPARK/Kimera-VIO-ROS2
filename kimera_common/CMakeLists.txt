cmake_minimum_required(VERSION 3.5)
project(kimera_common NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
    CONFIG_EXTRAS "kimera_common-extras.cmake"
)

install(
    DIRECTORY cmake
    DESTINATION share/${PROJECT_NAME}
)
