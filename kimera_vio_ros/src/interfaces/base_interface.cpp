#include <chrono>

#include "kimera_vio_ros/interfaces/base_interface.hpp"

using namespace std::chrono_literals;

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

  callback_group_pipeline_ = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

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
  if (this->pipeline_params_.parallel_run_) {
//    handle_shutdown_.get();
    handle_pipeline_.get();
  }
}

void BaseInterface::start()
{
  if (this->pipeline_params_.parallel_run_) {
    handle_pipeline_ = std::async(std::launch::async,
        &VIO::Pipeline::spin,
        pipeline_);
//    handle_shutdown_ = std::async(std::launch::async,
//        &VIO::Pipeline::shutdownWhenFinished,
//        pipeline_);
  } else {
    pipeline_timer_ = node_->create_wall_timer(
      10ms,
      std::bind(
        &VIO::Pipeline::spin,
        pipeline_),
      callback_group_pipeline_);
  }

}

}  // namespace interfaces
}  // namespace kimera_vio_ros
