#include "kimera_vio_ros/nodes/stereo_vio.hpp"

namespace kimera_vio_ros
{
namespace nodes
{

StereoVio::StereoVio()
: Node("stereo_vio"),
  kimera_vio_ros::interfaces::StereoInterface(*this),
  kimera_vio_ros::interfaces::ImuInterface(*this)
{
}

StereoVio::~StereoVio()
{
}

}  // namespace nodes
}  // namespace kimera_vio_ros
