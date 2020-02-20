#include "kimera_vio_ros/nodes/stereo_vio.hpp"

namespace kimera_vio_ros
{
namespace nodes
{

StereoVio::StereoVio()
: Node("stereo_vio"),
  interfaces::StereoInterface(*dynamic_cast<Node*>(this)),
  interfaces::ImuInterface(*dynamic_cast<Node*>(this))
{
}

StereoVio::~StereoVio()
{
}

}  // namespace nodes
}  // namespace kimera_vio_ros
