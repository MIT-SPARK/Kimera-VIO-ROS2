cmake_minimum_required(VERSION 3.5)
project(kimera_graph_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kimera_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

kimera_package()

set(msg_files
  "msg/PoseGraph.msg"
  "msg/PoseGraphEdge.msg"
  "msg/PoseGraphNode.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_package()
