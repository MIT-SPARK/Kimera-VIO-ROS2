#include "kimera_vio_ros/nodes/stereo_vio.hpp"

namespace kimera_vio_ros
{
namespace nodes
{

StereoVio::StereoVio(const rclcpp::NodeOptions & node_options)
: Node("stereo_vio", node_options),
  interfaces::StereoInterface(*dynamic_cast<Node*>(this)),
  interfaces::ImuInterface(*dynamic_cast<Node*>(this))
{
}

StereoVio::~StereoVio()
{
}

}  // namespace nodes
}  // namespace kimera_vio_ros
