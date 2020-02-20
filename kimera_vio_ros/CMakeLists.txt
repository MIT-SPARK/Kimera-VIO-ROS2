cmake_minimum_required(VERSION 3.5)
project(kimera_vio_ros)

# find_package(DBoW2 REQUIRED)
# find_package(Gflags REQUIRED)
# find_package(Glog 0.3.5 REQUIRED)
# find_package(opengv REQUIRED)
#find_package(pcl_ros)
find_package(GTSAM REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
# find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(image_transport)
find_package(kimera_common REQUIRED)
find_package(kimera_vio REQUIRED)
find_package(message_filters)
# find_package(nav_msgs REQUIRED)
# find_package(pcl_conversions)
# find_package(pcl_msgs)
# find_package(pose_graph_msgs)
# find_package(pose_graph_visualizer)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

kimera_package()

include_directories(
  include
  # ${PCL_COMMON_INCLUDE_DIRS}
)

set(dependencies
  # geometry_msgs
  # nav_msgs
  # pcl_msgs
  # pose_graph_msgs
  sensor_msgs
  # std_msgs
  # gtsam
  cv_bridge
  # tf2_ros
  kimera_vio
  # pcl_conversions
  # pcl_ros
  # pose_graph_visualizer
  rclcpp
  message_filters
  image_transport
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/interfaces/base_interface.cpp
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
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
