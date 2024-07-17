#include "ros2_helper_libs/position_tracker.hpp"
namespace position_tracking {

PositionTracker::PositionTracker(const std::string &sub,const rclcpp::NodeOptions &options): Node("position_tracker",options){
  using std::placeholders::_1;
    const std::string sub_topic =
        this->declare_parameter<std::string>("tracker_odom_topic", sub);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_topic, 10, std::bind(&PositionTracker::sub_callback_, this, _1));
}

double PositionTracker::polar_from_here(const geometry_msgs::msg::Point &cartesian){
  const geometry_msgs::msg::Point target = this->vector_from_here(cartesian);
  double ret_angle = std::atan2(target.y,target.x);
  RCLCPP_INFO(this->get_logger(),"Yaw angle relative to robot is %.2f rad",ret_angle);
  return ret_angle;
}

geometry_msgs::msg::Point PositionTracker::vector_from_here(const geometry_msgs::msg::Point &cartesian){
  geometry_msgs::msg::Point target = translate_waypoint(cartesian);
  tf_utils::convert_to_local(target, cartesian, this->cur_pose_);
  RCLCPP_INFO(this->get_logger(),"Transformed [%.2f x , %.2f y] to local frame: [%.2f x, %.2f y]",cartesian.x,cartesian.y,target.x,target.y);
  return target;
}

const geometry_msgs::msg::Pose PositionTracker::get_current_pose(){return this->cur_pose_;}

void PositionTracker::set_start(){ 
    this->set_start(this->cur_pose_);
}

void PositionTracker::set_start(const geometry_msgs::msg::Pose &start_pose){ 
    this->start_pose_ = start_pose;
    RCLCPP_INFO(this->get_logger(),"Using [%.2f x, %.2f y] yaw %.2f rad as tracker coordinate frame",this->start_pose_.position.x,this->start_pose_.position.y,tf_utils::get_yaw(this->start_pose_.orientation));
}

void PositionTracker::sub_callback_(const nav_msgs::msg::Odometry::SharedPtr msg){
    const geometry_msgs::msg::Pose new_pose = msg->pose.pose;
    this->cur_pose_ = new_pose;
}

geometry_msgs::msg::Point PositionTracker::translate_waypoint(const geometry_msgs::msg::Point &cartesian){
    geometry_msgs::msg::Point target;
    tf_utils::convert_to_global(cartesian, target, this->start_pose_);
    return target;
}

} //namespace position_tracking