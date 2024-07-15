#ifndef ROS2_HELPERS__TF2_UTILS_HPP__
#define ROS2_HELPERS__TF2_UTILS_HPP__

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "rclcpp/logging.hpp"
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
double radToDeg(const double &rad);

//! converts degrees to radians
double degToRad(const double &deg);


//! corrects angle if over/under +/- Pi radians
double fix_angle(const double &raw_angle);

//! returns yaw of rotation quaternion
double get_yaw(const geometry_msgs::msg::Quaternion &q_in);


//! returns cartesian coordinates (x,y) from polar (r,theta)
geometry_msgs::msg::Point convert_polar(const double &r, const double &theta);

//! transform point into local frame
void transform_to_local(geometry_msgs::msg::Point &target, const geometry_msgs::msg::Pose &local_frame);

//! convert point into local frame
void convert_to_local(geometry_msgs::msg::Point &local, const geometry_msgs::msg::Point &global, const geometry_msgs::msg::Pose &local_frame);

//! convert point into global frame
void convert_to_global(const geometry_msgs::msg::Point &local, geometry_msgs::msg::Point &global, const geometry_msgs::msg::Pose &local_frame);


//! returns polar angle between two points (from point1 to point2)
double get_polar_angle(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2);


//! returns Euclidean distance between two points 
double get_euclidean_distance(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2);


//! returns normal of line segment (point1, point2)
double get_normal(const geometry_msgs::msg::Point &point1,
                     const geometry_msgs::msg::Point &point2);


//! returns smallest angle between 2 orientations
double smallest_angle(const double &theta1, const double &theta2);

} // namespace tf_utils

#endif