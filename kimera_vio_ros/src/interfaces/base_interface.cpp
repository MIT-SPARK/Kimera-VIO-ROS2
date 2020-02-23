#include "kimera_vio_ros/interfaces/base_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BaseInterface::BaseInterface(
  rclcpp::Node & node)
: VIO::DataProviderInterface(),
  node_(node),
  pipeline_(nullptr)
{
  base_link_frame_id_ = node_.declare_parameter(
    "frame_id.base_link", "base_link");
  map_frame_id_ = node_.declare_parameter(
    "frame_id.map", "map");
  world_frame_id_ = node_.declare_parameter(
    "frame_id.world", "world");
  
  pipeline_.reset(new VIO::Pipeline(this->pipeline_params_));
}

BaseInterface::~BaseInterface()
{
  pipeline_->shutdown();
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
