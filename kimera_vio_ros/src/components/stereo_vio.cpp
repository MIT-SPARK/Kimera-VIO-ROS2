#include "kimera_vio_ros/components/stereo_vio.hpp"

namespace kimera_vio_ros
{
namespace components
{

StereoVio::StereoVio(
  const rclcpp::NodeOptions & options)
: Node("stereo_vio", options),
  interfaces::StereoInterface(*dynamic_cast<Node *>(this)),
  interfaces::ImuInterface(*dynamic_cast<Node *>(this))
{
}

StereoVio::~StereoVio()
{
}

}  // namespace components
}  // namespace kimera_vio_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kimera_vio_ros::components::StereoVio)
