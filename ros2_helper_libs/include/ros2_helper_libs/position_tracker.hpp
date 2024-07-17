#ifndef ROS2_HELPERS__POSITION_TRACKER_HPP__
#define ROS2_HELPERS__POSITION_TRACKER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ros2_helper_libs/tf2_utils.hpp"

namespace position_tracking {

class PositionTracker : public rclcpp::Node {

public:
  PositionTracker(const std::string &sub = "/odometry/filtered",const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  //! return polar angle relative to robot
  double polar_from_here(const geometry_msgs::msg::Point &cartesian);

  //! translate position from relative to start pose to current position
  geometry_msgs::msg::Point vector_from_here(const geometry_msgs::msg::Point &cartesian);

  //! return robot pose in odometry frame
  const geometry_msgs::msg::Pose get_current_pose();

  //! set the start pose to surrent position
  void set_start();

  //! set the start pose to given pose
  void set_start(const geometry_msgs::msg::Pose &start_pose);

protected:

void sub_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);

//! translate waypoint from start pose coordinate to odom coordinate frame 
geometry_msgs::msg::Point translate_waypoint(const geometry_msgs::msg::Point &cartesian);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  geometry_msgs::msg::Pose cur_pose_, start_pose_;
};

} //namespace position_tracking

#endif