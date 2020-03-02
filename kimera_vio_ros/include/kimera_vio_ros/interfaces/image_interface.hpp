#ifndef KIMERA_ROS__INTERFACES__IMAGE_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__IMAGE_INTERFACE_HPP_

#include "cv_bridge/cv_bridge.h"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;

namespace kimera_vio_ros
{
namespace interfaces
{

class ImageInterface : virtual public BaseInterface
{
public:
  ImageInterface(
    rclcpp::Node::SharedPtr & node);
  virtual ~ImageInterface();

protected:
  const cv::Mat readRosImage(const Image::ConstSharedPtr & img_msg);

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__IMAGE_INTERFACE_HPP_
