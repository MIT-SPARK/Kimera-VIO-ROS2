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

class StereoInterface : public ImageInterface
{
public:
  StereoInterface(
    rclcpp::Node & node);
  ~StereoInterface();

private:
  void stereo_cb(
    const Image::SharedPtr left_msg,
    const Image::SharedPtr right_msg);

private:
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_stereo_;
  typedef message_filters::sync_policies::ExactTime<Image, Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<message_filters::Subscriber<Image>> l_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> r_sub_;
  // image_transport::SubscriberFilter left_sub_, right_sub_;

  VIO::FrameId frame_count_;
  rclcpp::Time last_stereo_timestamp_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__STEREO_INTERFACE_HPP_
