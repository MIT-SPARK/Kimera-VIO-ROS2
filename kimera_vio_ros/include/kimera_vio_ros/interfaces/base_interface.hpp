#ifndef KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_

#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "rclcpp/rclcpp.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

class BaseInterface : public VIO::DataProviderInterface
{
public:
  BaseInterface(
    rclcpp::Node & node);
  virtual ~BaseInterface();

protected:
  rclcpp::Node & node_;
  VIO::Pipeline::Ptr pipeline_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__BASE_INTERFACE_HPP_
