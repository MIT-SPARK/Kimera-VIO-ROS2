#include "kimera_graph_visualizer/visualizer.hpp"
#include "interactive_markers/menu_handler.hpp"

using std::placeholders::_1;

Visualizer::Visualizer()
: Node("visualizer")
{
  RCLCPP_INFO(this->get_logger(), "Initializing pose graph visualizer");

  // get parameters
  if (!this->get_parameter("frame_id", frame_id_)) {
    RCLCPP_ERROR(this->get_logger(), "Did not load frame id");
  }

  // start subscribers
  pose_graph_sub_ = this->create_subscription<kimera_graph_msgs::msg::PoseGraph>(
    "graph", 10, std::bind(&Visualizer::PoseGraphCallback, this, _1));

  // start publishers
  odometry_edge_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "odometry_edges", 10);
  loop_edge_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "loop_edges", 10);
  rejected_loop_edge_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "rejected_loop_edges", 10);

  graph_node_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "graph_nodes", 10);
  graph_node_id_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "graph_nodes_ids", 10);

  interactive_mrkr_srvr_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_node", this);
}

void Visualizer::PoseGraphCallback(const kimera_graph_msgs::msg::PoseGraph::SharedPtr msg)
{
  // iterate through nodes in pose graph
  for (const kimera_graph_msgs::msg::PoseGraphNode & msg_node : msg->nodes) {
    // Fill pose nodes (representing the robot position)
    keyed_poses_[msg_node.key] = msg_node.pose;
  }

  // iterate through edges in pose graph
  for (const kimera_graph_msgs::msg::PoseGraphEdge & msg_edge : msg->edges) {
    if (msg_edge.type == kimera_graph_msgs::msg::PoseGraphEdge::ODOM) {
      odometry_edges_.emplace_back(
        std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == kimera_graph_msgs::msg::PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(
        std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == kimera_graph_msgs::msg::PoseGraphEdge::REJECTED_LOOPCLOSE) {
      rejected_loop_edges_.emplace_back(
        std::make_pair(msg_edge.key_from, msg_edge.key_to));
    }
  }

  visualize();
}

geometry_msgs::msg::Point Visualizer::getPositionFromKey(
  long unsigned int key) const
{
  auto v = keyed_poses_.at(key);
  geometry_msgs::msg::Point p;
  p.x = v.position.x;
  p.y = v.position.y;
  p.z = v.position.z;
  return p;
}

// Interactive Marker Menu to click and see key of node
void Visualizer::MakeMenuMarker(
  const geometry_msgs::msg::Pose & position,
  const std::string & id_number)
{
  interactive_markers::MenuHandler menu_handler;

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.scale = 1.0;
  int_marker.pose = position;
  int_marker.name = id_number;

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  interactive_mrkr_srvr_->insert(int_marker);
  menu_handler.apply(*interactive_mrkr_srvr_, int_marker.name);
}


void Visualizer::visualize()
{
  // Publish odometry edges.
  if (odometry_edge_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;

    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      const auto key1 = odometry_edges_[ii].first;
      const auto key2 = odometry_edges_[ii].second;

      m.points.push_back(getPositionFromKey(key1));
      m.points.push_back(getPositionFromKey(key2));
    }
    odometry_edge_pub_->publish(m);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 1;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.color.r = 0.0;
    m.color.g = 0.2;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
      const auto key1 = loop_edges_[ii].first;
      const auto key2 = loop_edges_[ii].second;

      m.points.push_back(getPositionFromKey(key1));
      m.points.push_back(getPositionFromKey(key2));
    }
    loop_edge_pub_->publish(m);
  }

  // Publish the rejected loop closure edges
  if (rejected_loop_edge_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 1;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.color.r = 0.5;
    m.color.g = 0.5;
    m.color.b = 0.5;
    m.color.a = 0.7;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;

    for (size_t ii = 0; ii < rejected_loop_edges_.size(); ++ii) {
      const auto key1 = rejected_loop_edges_[ii].first;
      const auto key2 = rejected_loop_edges_[ii].second;

      m.points.push_back(getPositionFromKey(key1));
      m.points.push_back(getPositionFromKey(key2));
    }
    rejected_loop_edge_pub_->publish(m);
  }

  // Publish node IDs in the pose graph.
  if (graph_node_id_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;

    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    m.scale.z = 0.01; // Only Scale z is used - height of capital A in the text
    m.pose.orientation.w = 1.0;

    int id_base = 100;
    for (const auto & keyedPose : keyed_poses_) {
      m.pose = keyedPose.second;
      // Display text for the node
      m.text = std::to_string(keyedPose.first);
      m.id = id_base + keyedPose.first;
      graph_node_id_pub_->publish(m);
    }

    // publish the interactive click-and-see key markers
    for (const auto & keyedPose : keyed_poses_) {
      std::string robot_id = std::to_string(keyedPose.first);
      MakeMenuMarker(keyedPose.second, robot_id);
    }
    if (interactive_mrkr_srvr_ != nullptr) {
      interactive_mrkr_srvr_->applyChanges();
    }
  }

  // Publish keyframe nodes in the pose graph.
  if (graph_node_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 4;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.3;
    m.color.a = 0.8;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.pose.orientation.w = 1.0;

    for (const auto & keyedPose : keyed_poses_) {
      m.points.push_back(getPositionFromKey(keyedPose.first));
    }
    graph_node_pub_->publish(m);
  }
}
