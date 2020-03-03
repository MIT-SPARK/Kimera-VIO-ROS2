#include "kimera_vio_ros/interfaces/base_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BaseInterface::BaseInterface(
  rclcpp::Node::SharedPtr & node)
: VIO::DataProviderInterface(),
  node_(nullptr),
  pipeline_(nullptr)
{
  node_ = node;

  base_link_frame_id_ = node->declare_parameter(
    "frame_id.base_link", "base_link");
  map_frame_id_ = node->declare_parameter(
    "frame_id.map", "map");
  world_frame_id_ = node->declare_parameter(
    "frame_id.world", "world");

  pipeline_.reset(new VIO::Pipeline(this->pipeline_params_));
}

BaseInterface::~BaseInterface()
{
  pipeline_->shutdown();
  handle_pipeline_.get();
}

void BaseInterface::start()
{
  handle_pipeline_ = std::async(std::launch::async,
                                &VIO::Pipeline::spin,
                                pipeline_);
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
