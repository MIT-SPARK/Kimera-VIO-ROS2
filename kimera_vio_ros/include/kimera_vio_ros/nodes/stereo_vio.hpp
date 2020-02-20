#ifndef KIMERA_ROS__NODES__STEREO_VIO_HPP_
#define KIMERA_ROS__NODES__STEREO_VIO_HPP_

#include "kimera_vio_ros/interfaces/imu_interface.hpp"
#include "kimera_vio_ros/interfaces/stereo_interface.hpp"

namespace kimera_vio_ros
{
namespace nodes
{

class StereoVio
  : public rclcpp::Node,
  public interfaces::StereoInterface,
  public interfaces::ImuInterface
{
public:
  StereoVio(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~StereoVio();

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__NODES__STEREO_VIO_HPP_
