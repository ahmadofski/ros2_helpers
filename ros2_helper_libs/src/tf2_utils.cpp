#include "ros2_helper_libs/tf2_utils.hpp"


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


//! corrects angle if over/under +/- Pi radians
double fix_angle(const double &raw_angle) {
  if (raw_angle > M_PI) {
    return raw_angle - 2 * M_PI;
  } else if (raw_angle < -M_PI) {
   return raw_angle + 2 * M_PI;
  }
  return raw_angle;
}


//! returns yaw of rotation quaternion
double get_yaw(const geometry_msgs::msg::Quaternion &q_in) {
  tf2::Quaternion q;
  tf2::fromMsg(q_in, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return fix_angle(yaw);
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
  
  global.x = local.x * std::cos(yaw) - local.y * std::sin(yaw) + local_frame.position.x;
  global.y = local.x * std::sin(yaw) + local.y * std::cos(yaw) + local_frame.position.y;
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
  return std::fmod(fix_angle(theta1 - theta2), 2 * M_PI);
}


} // namespace tf_utils

