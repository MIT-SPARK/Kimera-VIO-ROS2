#include "kimera_vio_ros/interfaces/backend_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

BackendInterface::BackendInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node),
  backend_output_queue_("Backend output"),
  tf_broadcaster_{node_}
{
  pipeline_->registerBackendOutputCallback(
    std::bind(
      // &BackendInterface::callbackBackendOutput,
      &BackendInterface::publishBackendOutput,
      this,
      std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  odometry_pub_ = node_->create_publisher<Odometry>("odometry", qos);
  pointcloud_pub_ = node_->create_publisher<PointCloud2>("time_horizon_pointcloud", qos);
}

BackendInterface::~BackendInterface()
{
  backend_output_queue_.shutdown();
}

void BackendInterface::publishBackendOutput(
  const VIO::BackendOutput::Ptr & output)
{
  CHECK(output);
  publishTf(output);
  if (odometry_pub_->get_subscription_count() > 0) {
    publishState(output);
  }
  // if (imu_bias_pub_.getNumSubscribers() > 0) {
  //   publishImuBias(output);
  // }
   if (pointcloud_pub_->get_subscription_count() > 0) {
     publishTimeHorizonPointCloud(output);
   }
}


void BackendInterface::publishState(
  const VIO::BackendOutput::Ptr & output) const
{
  CHECK(output);
  // Get latest estimates for odometry.
  const VIO::Timestamp & ts = output->timestamp_;
  const gtsam::Pose3 & pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3 & rotation = pose.rotation();
  const gtsam::Quaternion & quaternion = rotation.toQuaternion();
  const gtsam::Vector3 & velocity = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6 & pose_cov =
    gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3 & vel_cov =
    gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  // First publish odometry estimate
  Odometry odometry_msg;

  // Create header.
  odometry_msg.header.stamp = rclcpp::Time(ts);
  odometry_msg.header.frame_id = world_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;

  // Position
  odometry_msg.pose.pose.position.x = pose.x();
  odometry_msg.pose.pose.position.y = pose.y();
  odometry_msg.pose.pose.position.z = pose.z();

  // Orientation
  odometry_msg.pose.pose.orientation.w = quaternion.w();
  odometry_msg.pose.pose.orientation.x = quaternion.x();
  odometry_msg.pose.pose.orientation.y = quaternion.y();
  odometry_msg.pose.pose.orientation.z = quaternion.z();

  // Remap covariance from GTSAM convention
  // to odometry convention and fill in covariance
  static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};

  // Position covariance first, angular covariance after
  DCHECK_EQ(pose_cov.rows(), remapping.size());
  DCHECK_EQ(pose_cov.rows() * pose_cov.cols(),
    odometry_msg.pose.covariance.size());
  for (int i = 0; i < pose_cov.rows(); i++) {
    for (int j = 0; j < pose_cov.cols(); j++) {
      odometry_msg.pose
      .covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
        pose_cov(i, j);
    }
  }

  // Linear velocities, trivial values for angular
  const gtsam::Matrix3 & inversed_rotation = rotation.transpose();
  const VIO::Vector3 velocity_body = inversed_rotation * velocity;
  odometry_msg.twist.twist.linear.x = velocity_body(0);
  odometry_msg.twist.twist.linear.y = velocity_body(1);
  odometry_msg.twist.twist.linear.z = velocity_body(2);

  // Velocity covariance: first linear
  // and then angular (trivial values for angular)
  const gtsam::Matrix3 vel_cov_body =
    inversed_rotation.matrix() * vel_cov * rotation.matrix();
  DCHECK_EQ(vel_cov_body.rows(), 3);
  DCHECK_EQ(vel_cov_body.cols(), 3);
  DCHECK_EQ(odometry_msg.twist.covariance.size(), 36);
  for (int i = 0; i < vel_cov_body.rows(); i++) {
    for (int j = 0; j < vel_cov_body.cols(); j++) {
      odometry_msg.twist
      .covariance[i * static_cast<int>(
          sqrt(odometry_msg.twist.covariance.size())) +
        j] = vel_cov_body(i, j);
    }
  }
  // Publish message
  odometry_pub_->publish(odometry_msg);
}

void BackendInterface::publishTf(const VIO::BackendOutput::Ptr & output)
{
  CHECK(output);

  const VIO::Timestamp & timestamp = output->timestamp_;
  const gtsam::Pose3 & pose = output->W_State_Blkf_.pose_;
  const gtsam::Quaternion & quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  TransformStamped odom_tf;
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

void BackendInterface::publishTimeHorizonPointCloud(
    const VIO::BackendOutput::Ptr& output) const {
  CHECK(output);
  const VIO::Timestamp & timestamp = output->timestamp_;
  const VIO::PointsWithIdMap& points_with_id =
      output->landmarks_with_id_map_;
  const VIO::LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map =
      output->lmk_id_to_lmk_type_map_;

  if (points_with_id.size() == 0) {
    // No points to visualize.
    return;
  }

  PointCloud2::UniquePtr pc_msg(new PointCloud2);
  pc_msg->header.stamp = rclcpp::Time(timestamp);
  pc_msg->header.frame_id = world_frame_id_;
  pc_msg->width = points_with_id.size();
  pc_msg->height = 1;
  pc_msg->is_dense = true;
  pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
  pc_msg->row_step = pc_msg->point_step * pc_msg->width;
  pc_msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(pc_msg->width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

  bool color_the_cloud = false;
  if (lmk_id_to_lmk_type_map.size() != 0) {
    color_the_cloud = true;
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  // Populate cloud structure with 3D points.
  size_t i = 0;
  for (const std::pair<VIO::LandmarkId, gtsam::Point3>& id_point : points_with_id) {
    const gtsam::Point3 point_3d = id_point.second;
    *iter_x = static_cast<float>(point_3d.x());
    *iter_y = static_cast<float>(point_3d.y());
    *iter_z = static_cast<float>(point_3d.z());

    if (color_the_cloud) {
      DCHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
             lmk_id_to_lmk_type_map.end());
      switch (lmk_id_to_lmk_type_map.at(id_point.first)) {
        case VIO::LandmarkType::SMART: {
          *iter_r = 0;
          *iter_g = 255;
          *iter_b = 0;
          break;
        }
        case VIO::LandmarkType::PROJECTION: {
          *iter_r = 0;
          *iter_g = 0;
          *iter_b = 255;
          break;
        }
        default: {
          *iter_r = 255;
          *iter_g = 0;
          *iter_b = 0;
          break;
        }
      }
    }
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
    i++;
  }
  pointcloud_pub_->publish(std::move(pc_msg));
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
