#include "kimera_vio_ros/components/stereo_vio.hpp"

namespace kimera_vio_ros
{
namespace components
{

StereoVio::StereoVio(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options),
  interfaces::StereoInterface(*dynamic_cast<Node *>(this)),
  interfaces::ImuInterface(*dynamic_cast<Node *>(this))
{
}

StereoVio::~StereoVio()
{
}

}  // namespace components
}  // namespace kimera_vio_ros
