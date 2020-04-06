#include "kimera_vio_ros/interfaces/imu_interface.hpp"

namespace kimera_vio_ros
{
namespace interfaces
{

ImuInterface::ImuInterface(
  rclcpp::Node::SharedPtr & node)
: BaseInterface(node),
  last_imu_timestamp_(0)
{
  this->registerImuSingleCallback(
    std::bind(
        &VIO::Pipeline::fillSingleImuQueue,
        vio_pipeline_.get(),
        std::placeholders::_1));

  this->registerImuMultiCallback(
    std::bind(
        &VIO::Pipeline::fillMultiImuQueue,
        vio_pipeline_.get(),
        std::placeholders::_1));

  callback_group_imu_ = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto imu_opt = rclcpp::SubscriptionOptions();
  imu_opt.callback_group = callback_group_imu_;

  std::string imu_topic = "imu";
  auto qos = rclcpp::SensorDataQoS();
  imu_sub_ = node->create_subscription<Imu>(
    imu_topic,
    qos,
    std::bind(
      &ImuInterface::imu_cb,
      this,
      std::placeholders::_1),
    imu_opt);
}

ImuInterface::~ImuInterface()
{
}

void ImuInterface::imu_cb(const Imu::SharedPtr imu_msg)
{
  rclcpp::Time stamp(imu_msg->header.stamp);
  if (stamp.nanoseconds() > last_imu_timestamp_.nanoseconds()) {
    VIO::Timestamp timestamp = stamp.nanoseconds();
    VIO::ImuAccGyr imu_accgyr;

    imu_accgyr(0) = imu_msg->linear_acceleration.x;
    imu_accgyr(1) = imu_msg->linear_acceleration.y;
    imu_accgyr(2) = imu_msg->linear_acceleration.z;
    imu_accgyr(3) = imu_msg->angular_velocity.x;
    imu_accgyr(4) = imu_msg->angular_velocity.y;
    imu_accgyr(5) = imu_msg->angular_velocity.z;

    this->imu_single_callback_(VIO::ImuMeasurement(timestamp, imu_accgyr));
    //   LOG_EVERY_N(INFO, 200) << "Done: KimeraVioNode::imu_cb";
  }
  last_imu_timestamp_ = stamp;
}

}  // namespace interfaces
}  // namespace kimera_vio_ros
