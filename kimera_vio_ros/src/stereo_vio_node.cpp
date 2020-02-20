#include "gflags/gflags.h"
#include "kimera_vio_ros/nodes/stereo_vio.hpp"

using namespace kimera_vio_ros::nodes;

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  int g_argc = g_args.size();
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&g_argc, &argv, true);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto stereo_vio_node = std::make_shared<StereoVio>("stereo_vio");
  executor.add_node(stereo_vio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
