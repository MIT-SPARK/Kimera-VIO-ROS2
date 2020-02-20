#include "kimera_vio_ros/interfaces/base_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BaseInterface::BaseInterface(
  VIO::DataProviderInterface::Ptr dpi,
  rclcpp::Node & node)
:
  dpi_(dpi),
  node_(node),
  pipeline_(nullptr)
{
  pipeline_.reset(new VIO::Pipeline(dpi_->pipeline_params_));
}

BaseInterface::~BaseInterface()
{
   pipeline_->shutdown();
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
