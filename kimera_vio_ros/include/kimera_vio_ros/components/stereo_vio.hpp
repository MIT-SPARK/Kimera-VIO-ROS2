#ifndef KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_
#define KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_

#include "kimera_vio_ros/interfaces/imu_interface.hpp"
#include "kimera_vio_ros/interfaces/stereo_interface.hpp"

namespace kimera_vio_ros
{
namespace components
{

class StereoVio
  : public rclcpp::Node,
  public interfaces::StereoInterface,
  public interfaces::ImuInterface
{
public:
  explicit StereoVio(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoVio();

};

}  // namespace components
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_
