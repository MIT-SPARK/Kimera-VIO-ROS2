#ifndef KIMERA_ROS__UTILS__GEOMETRY_HPP_
#define KIMERA_ROS__UTILS__GEOMETRY_HPP_

#include "geometry_msgs/msg/transform.hpp"
#include "glog/logging.h"
#include "gtsam/geometry/Pose3.h"

namespace kimera_vio_ros
{
namespace utils
{

void msgTFtoPose(const geometry_msgs::msg::Transform & tf, gtsam::Pose3 * pose);

void poseToMsgTF(const gtsam::Pose3 & pose, geometry_msgs::msg::Transform * tf);

}  // namespace utils
}  // namespace kimera_vio_ros

#endif  // KIMERA_ROS__UTILS__GEOMETRY_HPP_
