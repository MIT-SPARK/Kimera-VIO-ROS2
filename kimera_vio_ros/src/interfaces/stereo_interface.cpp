#include "kimera_vio_ros/interfaces/stereo_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

StereoInterface::StereoInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node),
  ImageInterface(node),
  frame_count_(VIO::FrameId(0)),
  last_stereo_timestamp_(0)
{
  this->registerLeftFrameCallback(
    std::bind(
        &VIO::Pipeline::fillLeftFrameQueue,
        vio_pipeline_.get(),
        std::placeholders::_1));

  this->registerRightFrameCallback(
    std::bind(
        &VIO::Pipeline::fillRightFrameQueue,
        vio_pipeline_.get(),
        std::placeholders::_1));

  callback_group_stereo_ = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto stereo_opt = rclcpp::SubscriptionOptions();
  stereo_opt.callback_group = callback_group_stereo_;

  int queue_size_ = 10;

  auto info_qos = rclcpp::SystemDefaultsQoS();
  std::string left_info_topic = "left/camera_info";
  std::string right_info_topic = "right/camera_info";
  left_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
      node_,
      left_info_topic,
      info_qos.get_rmw_qos_profile());
  right_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
      node_,
      right_info_topic,
      info_qos.get_rmw_qos_profile());
  exact_info_sync_ = std::make_shared<ExactInfoSync>(
      ExactInfoPolicy(queue_size_), *left_info_sub_, *right_info_sub_);
  exact_info_sync_->registerCallback(&StereoInterface::camera_info_cb, this);



  auto image_qos = rclcpp::SensorDataQoS();
  std::string left_image_topic = "left/image";
  std::string right_image_topic = "right/image";

  // TODO: Perhaps switch to image_transport to support more transports
  // std::string transport = "raw";
  // image_transport::TransportHints hints(node, transport);
  // left_sub_.subscribe(node, left_topic, hints.getTransport(), qos.get_rmw_qos_profile());
  // right_sub_.subscribe(node, right_topic, hints.getTransport(), qos.get_rmw_qos_profile());
  // exact_image_sync_ = std::make_shared<ExactImageSync>(
  //   ExactImagePolicy(queue_size_), left_sub_, right_sub_);

  // TODO: Assign message filter subscribers to callback_group_stereo_
  // Pending: https://github.com/ros2/message_filters/issues/45
  left_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
    node_,
    left_image_topic,
    image_qos.get_rmw_qos_profile());
  right_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
    node_,
    right_image_topic,
    image_qos.get_rmw_qos_profile());
  exact_image_sync_ = std::make_shared<ExactImageSync>(
    ExactImagePolicy(queue_size_), *left_image_sub_, *right_image_sub_);
  exact_image_sync_->registerCallback(&StereoInterface::stereo_cb, this);
}

StereoInterface::~StereoInterface()
{
}

void StereoInterface::camera_info_cb(
    const CameraInfo::ConstSharedPtr & left_msg,
    const CameraInfo::ConstSharedPtr & right_msg) {
  CHECK_GE(vio_params_->camera_params_.size(), 2u);

  // Initialize CameraParams for pipeline.
  msgCamInfoToCameraParams(
      left_msg, &vio_params_->camera_params_.at(0));
  msgCamInfoToCameraParams(
      right_msg, &vio_params_->camera_params_.at(1));

  vio_params_->camera_params_.at(0).print();
  vio_params_->camera_params_.at(1).print();

  // Unregister this callback as it is no longer needed.
  RCLCPP_INFO(node_->get_logger(),
      "Unregistering CameraInfo subscribers as data has been received.");
  left_info_sub_->unsubscribe();
  right_info_sub_->unsubscribe();

  // Signal the correct reception of camera info
  camera_info_received_ = true;
}

void StereoInterface::stereo_cb(
  const Image::SharedPtr left_msg,
  const Image::SharedPtr right_msg)
{
  rclcpp::Time left_stamp(left_msg->header.stamp);
  rclcpp::Time right_stamp(right_msg->header.stamp);
  if (left_stamp.nanoseconds() > last_stereo_timestamp_.nanoseconds()) {
    static const VIO::CameraParams & left_cam_info =
      vio_params_->camera_params_.at(0);
    static const VIO::CameraParams & right_cam_info =
      vio_params_->camera_params_.at(1);

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
