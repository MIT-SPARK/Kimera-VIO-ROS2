#include "kimera_vio_ros/interfaces/base_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BaseInterface::BaseInterface(
  // VIO::DataProviderInterface & dpi,
  rclcpp::Node & node)
:
  // dpi_(),
//  pipeline_(dpi.pipeline_params_),
  node_(node)  
{

//  pipeline_(dpi_.pipeline_params_);

}

BaseInterface::~BaseInterface()
{
  // pipeline_.shutdown();
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
