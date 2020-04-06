#include "kimera_vio_ros/utils/geometry.hpp"

namespace kimera_vio_ros
{
namespace utils
{

void msgTFtoPose(const geometry_msgs::msg::Transform& tf, gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);

  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void poseToMsgTF(const gtsam::Pose3& pose, geometry_msgs::msg::Transform* tf) {
  CHECK_NOTNULL(tf);

  tf->translation.x = pose.x();
  tf->translation.y = pose.y();
  tf->translation.z = pose.z();
  const gtsam::Quaternion& quat = pose.rotation().toQuaternion();
  tf->rotation.w = quat.w();
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
}

}  // namespace utils
}  // namespace kimera_vio_ros
