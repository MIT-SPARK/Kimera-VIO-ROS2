#ifndef KIMERA_ROS__INTERFACES__IMU_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__IMU_INTERFACE_HPP_

#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace sensor_msgs::msg;

namespace kimera_vio_ros
{
namespace interfaces
{

class ImuInterface: public BaseInterface
{
public:
  ImuInterface(
    rclcpp::Node & node);
  ~ImuInterface();

private:
  void imu_cb(const Imu::SharedPtr imu_msg);

private:
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_imu_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Time last_imu_timestamp_;
};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__IMU_INTERFACE_HPP_
