#ifndef KIMERA_ROS__INTERFACES__STEREO_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__STEREO_INTERFACE_HPP_

#include "kimera_vio_ros/interfaces/image_interface.hpp"
// #include "image_transport/subscriber_filter.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

namespace kimera_vio_ros
{
namespace interfaces
{

class StereoInterface : virtual public ImageInterface
{
public:
  StereoInterface(
    rclcpp::Node::SharedPtr & node);
  virtual ~StereoInterface();

private:
  void stereo_info_cb(
    const CameraInfo::ConstSharedPtr & left_msg,
    const CameraInfo::ConstSharedPtr & right_msg);
  void stereo_image_cb(
    const Image::SharedPtr left_msg,
    const Image::SharedPtr right_msg);

private:
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_stereo_;

  typedef message_filters::sync_policies::ExactTime<Image, Image> ExactImagePolicy;
  typedef message_filters::Synchronizer<ExactImagePolicy> ExactImageSync;
  std::shared_ptr<ExactImageSync> exact_image_sync_;
  std::shared_ptr<message_filters::Subscriber<Image>> left_image_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> right_image_sub_;
  // image_transport::SubscriberFilter left_sub_, right_sub_;

  typedef message_filters::sync_policies::ExactTime<CameraInfo, CameraInfo> ExactInfoPolicy;
  typedef message_filters::Synchronizer<ExactInfoPolicy> ExactInfoSync;
  std::shared_ptr<ExactInfoSync> exact_info_sync_;
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> left_info_sub_;
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> right_info_sub_;
  bool camera_info_received_ = false;

  VIO::FrameId frame_count_;
  rclcpp::Time last_stereo_timestamp_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__STEREO_INTERFACE_HPP_
