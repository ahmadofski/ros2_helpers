#ifndef ROS2_HELPERS__GEOMETRY_MSGS_UTILS_HPP__
#define ROS2_HELPERS__GEOMETRY_MSGS_UTILS_HPP__

#include "geometry_msgs/msg/detail/point__struct.hpp"
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
#include <vector>

using namespace geometry_msgs::msg; 

namespace construct_utils {
Point make_point(const double &x = 0.0, const double &y = 0.0, const double &z = 0.0);

Quaternion make_quaternion(const double &w = 1.0, const double &x = 0.0, const double &y = 0.0, const double &z = 0.0);

Quaternion make_quaternion(const tf2::Quaternion &q_in);

Quaternion quaternion_from_rpy(const double &roll, const double &pitch, const double &yaw);

Pose make_pose(const Point &p, const Quaternion &q);

Pose make_pose(const double &p_x, const double &p_y, const double &p_z, const double &q_w = 1.0,
               const double &q_x = 0.0, const double &q_y = 0.0, const double &q_z = 0.0);


} // namespace construct_utils

namespace msg_operators {

Point scalar_mult(const Point &point, const double &scalar);

Point scalar_add(const Point &point, const double &scalar);

Point vec_add(const Point &lhs, const Point &rhs);

Point vec_subt(const Point &lhs, const Point &rhs);

template<class BinaryOperator>
inline void apply_to_members(BinaryOperator op, const Point &lhs, const Point &rhs, Point &res){
    res.x = op(lhs.x,rhs.x);
    res.y = op(lhs.y,rhs.y);
    res.z = op(lhs.z,rhs.z);
}

template<class BinaryOperator>
inline Point apply_to_members(BinaryOperator op, const Point &lhs, const Point &rhs){
    Point res;
    apply_to_members(op,lhs,rhs,res);
    return res;
}

Quaternion ham_mult(const Quaternion &q1,const Quaternion &q2);

double dot(const Quaternion &q1,const Quaternion &q2);

double point_magnitude(const Point &p);

Quaternion yaw_rotate(const double &yaw);

Pose offset_pose(const Pose &pose, const Pose &offset);

Point centroid(const std::vector<Point> &points);

Point polygon_normal(const std::vector<Point> &points, const bool &ccw = false);

Quaternion polygon_normal_q(const std::vector<Point> &points, const bool &ccw = false);

void normalise(Quaternion &q);

Pose average_pose(const std::vector<Pose> &poses);

} // namespace msg_operators

#endif
