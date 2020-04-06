#ifndef KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_
#define KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kimera_vio_ros/interfaces/base_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

namespace kimera_vio_ros
{
namespace interfaces
{

class BackendInterface : virtual public BaseInterface
{
public:
  BackendInterface(
    rclcpp::Node::SharedPtr & node);
  virtual ~BackendInterface();

public:
  inline void callbackBackendOutput(const VIO::BackendOutput::Ptr & output)
  {
    backend_output_queue_.push(output);
  }

protected:
  void publishBackendOutput(const VIO::BackendOutput::Ptr & output);

protected:
  VIO::ThreadsafeQueue<VIO::BackendOutput::Ptr> backend_output_queue_;

private:
  void publishState(const VIO::BackendOutput::Ptr & output) const;
  void publishTf(const VIO::BackendOutput::Ptr & output);
  void publishTimeHorizonPointCloud(const VIO::BackendOutput::Ptr & output) const;
  // void publishImuBias(const VIO::BackendOutput::Ptr& output) const;

private:
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;

};

}  // namespace interfaces
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__INTERFACES__BACKEND_INTERFACE_HPP_
