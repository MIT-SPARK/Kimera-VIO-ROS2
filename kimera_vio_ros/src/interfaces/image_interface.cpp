#include "kimera_vio_ros/interfaces/image_interface.hpp"
#include "kimera_vio_ros/utils/geometry.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

ImageInterface::ImageInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node)
{
}

ImageInterface::~ImageInterface()
{
}

void ImageInterface::msgCamInfoToCameraParams(
  const CameraInfo::ConstSharedPtr & cam_info,
  VIO::CameraParams * cam_params)
{
  CHECK_NOTNULL(cam_params);

  // Get intrinsics from incoming CameraInfo messages:
  cam_params->camera_id_ = cam_info->header.frame_id;
  CHECK(!cam_params->camera_id_.empty());

  CHECK(cam_info->distortion_model == "plumb_bob" ||
    cam_info->distortion_model == "equidistant");

  if (cam_info->distortion_model == "plumb_bob") {
    // Kimera-VIO terms the plumb bob dist. model the as radtan.
    cam_params->distortion_model_ = "radtan";
    // Kimera-VIO can't take a 6th order radial distortion term.
    CHECK_EQ(cam_info->d.size(), 5);
  } else {
    cam_params->distortion_model_ = cam_info->distortion_model;
    CHECK_EQ(cam_info->d.size(), 4);
  }

  const std::vector<double> & distortion_coeffs =
    std::vector<double>(cam_info->d.begin(), cam_info->d.begin() + 4);

  CHECK_EQ(distortion_coeffs.size(), 4);
  VIO::CameraParams::convertDistortionVectorToMatrix(
    distortion_coeffs, &cam_params->distortion_coeff_);

  cam_params->image_size_ = cv::Size(cam_info->width, cam_info->height);

  cam_params->frame_rate_ = 0;  // TODO(marcus): is there a way to get this?

  std::array<double, 4> intrinsics = {
    cam_info->k[0], cam_info->k[4], cam_info->k[2], cam_info->k[5]};
  cam_params->intrinsics_ = intrinsics;
  VIO::CameraParams::convertIntrinsicsVectorToMatrix(
    cam_params->intrinsics_, &cam_params->K_);

  VIO::CameraParams::createGtsamCalibration(cam_params->distortion_coeff_,
    cam_params->intrinsics_,
    &cam_params->calibration_);

  // Get extrinsics from the TF tree:
  static constexpr size_t kTfLookupTimeout = 5u;
  geometry_msgs::msg::TransformStamped cam_tf;

  try {
    cam_tf = tf_buffer_->lookupTransform(
      base_link_frame_id_,
      cam_params->camera_id_,
      cam_info->header.stamp,
      rclcpp::Duration(kTfLookupTimeout));
  } catch (tf2::TransformException & ex) {
    RCLCPP_FATAL(node_->get_logger(),
      "TF for left/right camera frames not available. Either publish to "
      "tree or provide CameraParameter yaml files.:\n%s", ex.what());
    rclcpp::shutdown();
  }

  utils::msgTFtoPose(cam_tf.transform, &cam_params->body_Pose_cam_);
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
