#ifndef ROS2_HELPERS__TF2_UTILS_HPP__
#define ROS2_HELPERS__TF2_UTILS_HPP__

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "rclcpp/rclcpp.hpp"

//select older library include if tagged pre ros-humble
#ifdef PREHUMBLE
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ros2_helper_libs/geometry_msgs_utils.hpp"
#include <cmath>




using namespace std::chrono_literals;
namespace tf_utils {

//! converts radians to degrees
double radToDeg(const double &rad) {
  return rad * 180 / M_PI;
}

//! converts degrees to radians
double degToRad(const double &deg) {
  return deg * M_PI / 180;
}

//! returns yaw of rotation quaternion
double get_yaw(const geometry_msgs::msg::Quaternion &q_in) {
  tf2::Quaternion q;
  tf2::fromMsg(q_in, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


//! returns cartesian coordinates (x,y) from polar (r,theta)
geometry_msgs::msg::Point convert_polar(const double &r, const double &theta) {
  geometry_msgs::msg::Point cartesian;
  cartesian.x = r * std::cos(theta);
  cartesian.y = r * std::sin(theta);
  return cartesian;
}

//! transform point into local frame
void transform_to_local(geometry_msgs::msg::Point &target, const geometry_msgs::msg::Pose &local_frame) {
  const double yaw = get_yaw(local_frame.orientation);
  const double x_d = target.x - local_frame.position.x;
  const double y_d = target.y - local_frame.position.y;
  target.x = x_d * std::cos(yaw) + y_d * sin(yaw);
  target.y = x_d * -std::sin(yaw) + y_d * cos(yaw);
}

//! convert point into local frame
void convert_to_local(geometry_msgs::msg::Point &local, const geometry_msgs::msg::Point &global, const geometry_msgs::msg::Pose &local_frame) {
  const double yaw = get_yaw(local_frame.orientation);
  const double x_d = global.x - local_frame.position.x;
  const double y_d = global.y - local_frame.position.y;
  local.x = x_d * std::cos(yaw) + y_d * sin(yaw);
  local.y = x_d * -std::sin(yaw) + y_d * cos(yaw);
}

//! convert point into global frame
void convert_to_global(const geometry_msgs::msg::Point &local, geometry_msgs::msg::Point &global, const geometry_msgs::msg::Pose &local_frame) {
  const double yaw = get_yaw(local_frame.orientation);
  
  global.x = local.x * std::cos(yaw) - local.y * sin(yaw) + local_frame.position.x;
  global.y = local.x * std::sin(yaw) + local.y * cos(yaw) + local_frame.position.y;
}


//! corrects angle if over/under +/- Pi radians
double fix_angle(const double &raw_angle) {
  if (raw_angle > M_PI) {
    return raw_angle - 2 * M_PI;
  } else if (raw_angle < -M_PI) {
   return raw_angle + 2 * M_PI;
  }
  return raw_angle;
}


//! returns polar angle between two points (from point1 to point2)
double get_polar_angle(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2) {
  return 
      std::atan2(point2.y - point1.y, point2.x - point1.x);
}


//! returns Euclidean distance between two points 
double get_euclidean_distance(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2) {
  return 
      std::sqrt(std::pow(point2.x - point1.x,2) + std::pow(point2.y - point1.y,2));
}


//! returns normal of line segment (point1, point2)
double get_normal(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2) {
 
  return fix_angle(get_polar_angle(point1,point2) + M_PI_2);
}


//! returns smallest angle between 2 orientations
double smallest_angle(const double &theta1, const double &theta2) {
  return fix_angle(std::fmod(theta1 - theta2, 2 * M_PI));
}


} // namespace tf_utils

#endif