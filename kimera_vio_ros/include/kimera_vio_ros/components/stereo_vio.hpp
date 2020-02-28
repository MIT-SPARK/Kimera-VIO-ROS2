#ifndef KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_
#define KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_

#include "kimera_vio_ros/interfaces/stereo_vio_interface.hpp"

namespace kimera_vio_ros
{
namespace components
{

class StereoVio : public rclcpp::Node
{
public:
  StereoVio(
    const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  StereoVio(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  virtual ~StereoVio();

private:
  void init();

  std::unique_ptr<interfaces::BaseInterface> vio_node_;
};

}  // namespace components
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__COMPONENTS__STEREO_VIO_HPP_
