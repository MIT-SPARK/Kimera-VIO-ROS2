#include "kimera_graph_visualizer/visualizer.hpp"

int main(int argc, char ** argv)
{
  // Initiallize visualizer
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualizer>());
  rclcpp::shutdown();
  return 0;
}
