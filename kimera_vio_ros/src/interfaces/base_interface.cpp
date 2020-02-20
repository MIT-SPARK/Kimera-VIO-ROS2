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
  pipeline_.reset(new VIO::Pipeline(this->pipeline_params_));
}

BaseInterface::~BaseInterface()
{
  pipeline_->shutdown();
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
