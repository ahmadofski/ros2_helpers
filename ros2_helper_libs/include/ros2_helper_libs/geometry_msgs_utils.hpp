#ifndef ROS2_HELPERS__GEOMETRY_MSGS_UTILS_HPP__
#define ROS2_HELPERS__GEOMETRY_MSGS_UTILS_HPP__

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

//select older library include if tagged pre ros-humble
#ifdef PREHUMBLE
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>

using namespace geometry_msgs::msg; 

namespace construct_utils {
auto make_point(const double &x = 0.0, const double &y = 0.0, const double &z = 0.0)  {
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

auto make_quaternion(const double &w = 1.0, const double &x = 0.0, const double &y = 0.0, const double &z = 0.0)  {
  Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}


auto make_quaternion(const tf2::Quaternion &q_in) {
  Quaternion q;
  tf2::convert(q_in,q);
  return q;
}

auto make_pose(const Point &p, const Quaternion &q)  {
  Pose pose;
  pose.position = p;
  pose.orientation = q;
  return pose;
}

auto make_pose(const double &p_x, const double &p_y, const double &p_z, const double &q_w = 1.0,
               const double &q_x = 0.0, const double &q_y = 0.0, const double &q_z = 0.0)  {
  return make_pose(make_point(p_x, p_y, p_z),
                   make_quaternion(q_w, q_x, q_y, q_z));
}


} // namespace construct_utils

namespace msg_operators {

Point scalar_mult(const Point &point, const double &scalar) {
  Point result;
  result.x = point.x * scalar;
  result.y = point.y * scalar;
  result.z = point.z * scalar;
  return result;
}

Point scalar_add(const Point &point, const double &scalar) {
  Point result;
  result.x = point.x + scalar;
  result.y = point.y + scalar;
  result.z = point.z + scalar;
  return result;
}

Point vec_add(const Point &lhs, const Point &rhs) {
  Point result;
  result.x = lhs.x + rhs.x;
  result.y = lhs.y + rhs.y;
  result.z = lhs.z + rhs.z;
  return result;
}

Point vec_subt(const Point &lhs, const Point &rhs) {
  Point result;
  result.x = lhs.x - rhs.x;
  result.y = lhs.y - rhs.y;
  result.z = lhs.z - rhs.z;
  return result;
}

Quaternion ham_mult(const Quaternion &q1,const Quaternion &q2) {
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w - q1.y * q2.z + q1.z * q2.y;
  result.y = q1.w * q2.y + q1.x * q2.z + q1.y * q2.w - q1.z * q2.x;
  result.z = q1.w * q2.z - q1.x * q2.y + q1.y * q2.x + q1.z * q2.w;
  return result;
}

double point_magnitude(const Point &p){
  return std::sqrt(std::pow(p.x,2)+ std::pow(p.y,2) + std::pow(p.z,2));
}

/*
double get_yaw(const Quaternion &q){
  const double test = q.x*q.y + q.z*q.w;
  if (test > 0.499) {return 2 * std::atan2(q.x,q.w);}
  if (test < -0.499) {return -2 * std::atan2(q.x,q.w);}
  return std::atan2( 2 * test, 1 - 2 * (std::pow(q.y,2) + std::pow(q.z,2)));
}
*/

Quaternion yaw_rotate(const double &yaw){
  const double cosines = std::pow(std::cos(0.5),2);

  Quaternion result;
  result.w = cosines * std::cos(yaw*0.5);
  result.x = 0.0;
  result.y = cosines * std::sin(yaw*0.5);
  result.z = 0.0;
  return result;
}

Pose offset_pose(const Pose &pose, const Pose &offset){
    return construct_utils::make_pose(vec_add(pose.position, offset.position),ham_mult(pose.orientation, offset.orientation));
}

} // namespace msg_operators

#endif