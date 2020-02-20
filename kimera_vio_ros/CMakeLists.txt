cmake_minimum_required(VERSION 3.5)
project(kimera_vio_ros)

find_package(Boost REQUIRED COMPONENTS
  chrono
  date_time
  filesystem
  regex
  serialization
  system
  thread
  timer
)
# find_package(Gflags REQUIRED)
# find_package(Glog REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(kimera_common REQUIRED)
find_package(kimera_vio REQUIRED)
find_package(message_filters)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

kimera_package()

include_directories(
  include
)

set(dependencies
  cv_bridge
  kimera_vio
  message_filters
  rclcpp
  sensor_msgs
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/interfaces/base_interface.cpp
  src/interfaces/image_interface.cpp
  src/interfaces/imu_interface.cpp
  src/interfaces/stereo_interface.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies})
target_link_libraries(${library_name}
  kimera_vio)

# set(executable_name kimera_vio_ros_node)
# add_executable(${executable_name}
#   src/KimeraVio.cpp
# )
# ament_target_dependencies(${executable_name}
#   ${dependencies})
# target_link_libraries(${executable_name}
#   ${library_name}
#   kimera_vio)


install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY param
#   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skips the linter checks for copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
