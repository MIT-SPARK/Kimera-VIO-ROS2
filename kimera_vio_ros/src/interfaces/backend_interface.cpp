#include "kimera_vio_ros/interfaces/backend_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BackendInterface::BackendInterface(
  rclcpp::Node & node)
: BaseInterface(node),
  backend_output_queue_("Backend output"),
  tf_broadcaster_{node}
{
  
}

BackendInterface::~BackendInterface()
{
  backend_output_queue_.shutdown();
}

void BackendInterface::publishBackendOutput(
    const VIO::BackendOutput::Ptr& output) {
  // CHECK_NOTNULL(output);
  publishTf(output);
  // if (odometry_pub_.getNumSubscribers() > 0) {
  //   publishState(output);
  // }
  // if (imu_bias_pub_.getNumSubscribers() > 0) {
  //   publishImuBias(output);
  // }
  // if (pointcloud_pub_.getNumSubscribers() > 0) {
  //   publishTimeHorizonPointCloud(output);
  // }
}


void BackendInterface::publishTf(const VIO::BackendOutput::Ptr& output) {
  // CHECK_NOTNULL(output);

  const VIO::Timestamp& timestamp = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = rclcpp::Time(timestamp);
  odom_tf.header.frame_id = world_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  odom_tf.transform.translation.x = pose.x();
  odom_tf.transform.translation.y = pose.y();
  odom_tf.transform.translation.z = pose.z();
  odom_tf.transform.rotation.w = quaternion.w();
  odom_tf.transform.rotation.x = quaternion.x();
  odom_tf.transform.rotation.y = quaternion.y();
  odom_tf.transform.rotation.z = quaternion.z();
  tf_broadcaster_.sendTransform(odom_tf);
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
