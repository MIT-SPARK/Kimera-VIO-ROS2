#include "kimera_vio_ros/interfaces/image_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

ImageInterface::ImageInterface(
  rclcpp::Node & node)
: BaseInterface(node)
{
}

ImageInterface::~ImageInterface()
{
}

const cv::Mat ImageInterface::readRosImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  cv_bridge::CvImageConstPtr cv_constptr;
  try {
    cv_constptr = cv_bridge::toCvShare(img_msg);
  } catch (cv_bridge::Exception & exception) {
    // RCLCPP_FATAL(this->get_logger(), "cv_bridge exception: %s", exception.what());
    // rclcpp::shutdown();
  }

  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    // LOG(WARNING) << "Converting image...";
    cv::cvtColor(cv_constptr->image, cv_constptr->image, cv::COLOR_BGR2GRAY);
  } else {
    // CHECK_EQ(cv_constptr->encoding, sensor_msgs::image_encodings::MONO8)
    //     << "Expected image with MONO8 or BGR8 encoding.";
  }

  return cv_constptr->image;
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
