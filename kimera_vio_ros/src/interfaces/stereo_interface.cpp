#include "cv_bridge/cv_bridge.h"
#include "kimera_vio_ros/interfaces/stereo_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

const cv::Mat readRosImage(
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

StereoInterface::StereoInterface(
  rclcpp::Node & node)
: BaseInterface(node),
  frame_count_(VIO::FrameId(0)),
  last_stereo_timestamp_(0)
{
  this->registerLeftFrameCallback(
    std::bind(
      &VIO::Pipeline::fillLeftFrameQueue,
      pipeline_,
      std::placeholders::_1));

  this->registerRightFrameCallback(
    std::bind(
      &VIO::Pipeline::fillRightFrameQueue,
      pipeline_,
      std::placeholders::_1));

  callback_group_stereo_ = node.create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto stereo_opt = rclcpp::SubscriptionOptions();
  stereo_opt.callback_group = callback_group_stereo_;

  std::string transport = "raw";
  std::string left_topic = "left_cam";
  std::string right_topic = "right_cam";
  auto qos = rclcpp::SensorDataQoS();
  int queue_size_ = 10;

  // image_transport::TransportHints hints(node, transport);
  // left_sub_.subscribe(node, left_topic, hints.getTransport(), qos.get_rmw_qos_profile());
  // right_sub_.subscribe(node, right_topic, hints.getTransport(), qos.get_rmw_qos_profile());
  // exact_sync_ = std::make_shared<ExactSync>(
  //   ExactPolicy(queue_size_), left_sub_, right_sub_);

  l_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
    node.shared_from_this(),
    left_topic,
    qos.get_rmw_qos_profile());
  r_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
    node.shared_from_this(),
    right_topic,
    qos.get_rmw_qos_profile());
  exact_sync_ = std::make_shared<ExactSync>(
    ExactPolicy(queue_size_), *l_sub_, *r_sub_);

  exact_sync_->registerCallback(&StereoInterface::stereo_cb, this);

}

StereoInterface::~StereoInterface()
{
}

void StereoInterface::stereo_cb(
  const Image::SharedPtr left_msg,
  const Image::SharedPtr right_msg)
{
  rclcpp::Time left_stamp(left_msg->header.stamp);
  rclcpp::Time right_stamp(right_msg->header.stamp);
  if (left_stamp.nanoseconds() > last_stereo_timestamp_.nanoseconds()) {
    static const VIO::CameraParams & left_cam_info =
      pipeline_params_.camera_params_.at(0);
    static const VIO::CameraParams & right_cam_info =
      pipeline_params_.camera_params_.at(1);

    const VIO::Timestamp & timestamp_left = left_stamp.nanoseconds();
    const VIO::Timestamp & timestamp_right = right_stamp.nanoseconds();

    // CHECK(left_frame_callback_)
    // << "Did you forget to register the left frame callback?";
    // CHECK(right_frame_callback_)
    // << "Did you forget to register the right frame callback?";
    //  TODO: Use RCLCPP_INFO inplace of CHECK?
    // RCLCPP_INFO(this->get_logger(), "Did you forget to register the right frame callback?");

    left_frame_callback_(VIO::make_unique<VIO::Frame>(
        frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));
    right_frame_callback_(VIO::make_unique<VIO::Frame>(
        frame_count_, timestamp_right, right_cam_info, readRosImage(right_msg)));
    // LOG_EVERY_N(INFO, 30) << "Done: KimeraVioNode::stereo_cb";
    frame_count_++;
  }
  last_stereo_timestamp_ = left_stamp;
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
