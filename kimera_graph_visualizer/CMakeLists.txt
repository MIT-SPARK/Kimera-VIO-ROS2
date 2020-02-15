cmake_minimum_required(VERSION 3.5)
project(kimera_graph_visualizer)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(kimera_common REQUIRED)
find_package(kimera_graph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

kimera_package()

include_directories(
  include
)

set(executable_name visualizer_node)
add_executable(${executable_name}
  src/visualizer.cpp
  src/visualizer_node.cpp
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/visualizer.cpp
)

set(dependencies
  geometry_msgs
  interactive_markers
  kimera_graph_msgs
  rclcpp
  visualization_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
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
