#ifndef KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_

#include "kimera_vio_ros/interfaces/backend_interface.hpp"
#include "kimera_vio_ros/interfaces/imu_interface.hpp"
#include "kimera_vio_ros/interfaces/stereo_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

class StereoVioInterface
    :
//  public ImageInterface,
  public StereoInterface,
  public ImuInterface,
  public BackendInterface
{
public:
  StereoVioInterface(
    rclcpp::Node & node);
  ~StereoVioInterface();

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__STEREO_VIO_INTERFACE_HPP_
