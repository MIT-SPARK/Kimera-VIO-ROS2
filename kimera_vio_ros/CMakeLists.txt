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
find_package(Gflags REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(glog REQUIRED)
find_package(kimera_common REQUIRED)
find_package(kimera_vio REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

kimera_package()

include_directories(
  include
)

set(dependencies
  cv_bridge
  geometry_msgs
  kimera_vio
  message_filters
  nav_msgs
  pcl_conversions
  pcl_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/components/stereo_vio.cpp
  src/interfaces/backend_interface.cpp
  src/interfaces/base_interface.cpp
  src/interfaces/image_interface.cpp
  src/interfaces/imu_interface.cpp
  src/interfaces/stereo_interface.cpp
  src/interfaces/stereo_vio_interface.cpp
  src/utils/geometry.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies})
rclcpp_components_register_nodes(${library_name}
  "kimera_vio_ros::components::StereoVio")
target_link_libraries(${library_name}
  kimera_vio)

add_executable(stereo_vio_node
  src/stereo_vio_node.cpp)
ament_target_dependencies(stereo_vio_node
  ${dependencies})
target_link_libraries(stereo_vio_node
  ${library_name})

set(executable_names
  stereo_vio_node
)

install(TARGETS ${executable_names} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skips the linter checks for copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(dependencies)
ament_export_include_directories(include)

ament_package()
