#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "kimera_graph_msgs/msg/pose_graph.hpp"
#include "kimera_graph_msgs/msg/pose_graph_edge.hpp"
#include "kimera_graph_msgs/msg/pose_graph_node.hpp"

#include <unordered_map>

class Visualizer : public rclcpp::Node
{
public:
  Visualizer();

  void visualize();

private:
  void PoseGraphCallback(const kimera_graph_msgs::msg::PoseGraph::SharedPtr msg);

  geometry_msgs::msg::Point getPositionFromKey(long unsigned int key) const;

  void MakeMenuMarker(const geometry_msgs::msg::Pose & position, const std::string & id_number);

private:
  std::string frame_id_;

  // subscribers
  rclcpp::Subscription<kimera_graph_msgs::msg::PoseGraph>::SharedPtr pose_graph_sub_;

  // publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odometry_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rejected_loop_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_node_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_node_id_pub_;

  typedef std::pair<long unsigned int, long unsigned int> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::unordered_map<long unsigned int, geometry_msgs::msg::Pose> keyed_poses_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_mrkr_srvr_;
};

#endif
