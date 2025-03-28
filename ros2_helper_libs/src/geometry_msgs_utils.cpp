#include "ros2_helper_libs/geometry_msgs_utils.hpp"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

using namespace geometry_msgs::msg; 

namespace construct_utils {
Point make_point(const double &x, const double &y, const double &z)  {
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Quaternion make_quaternion(const double &w, const double &x, const double &y, const double &z)  {
  Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

Quaternion make_quaternion(const tf2::Quaternion &q_in) {
  Quaternion q;
  tf2::convert(q_in,q);
  return q;
}

Quaternion quaternion_from_rpy(const double &roll, const double &pitch, const double &yaw){
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    return make_quaternion(quaternion);
}

Pose make_pose(const Point &p, const Quaternion &q)  {
  Pose pose;
  pose.position = p;
  pose.orientation = q;
  return pose;
}

Pose make_pose(const double &p_x, const double &p_y, const double &p_z, const double &q_w,
               const double &q_x, const double &q_y, const double &q_z)  {
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

double dot(const Quaternion &q1,const Quaternion &q2){
  return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
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

Point centroid (const std::vector<Point> &points){
  Point centroid = construct_utils::make_point();
  if (points.size() > 0) {
    const double divisor = 1.0 / points.size();
    for (auto point : points) {
      centroid.x += point.x * divisor;
      centroid.y += point.y * divisor;
      centroid.z += point.z * divisor;
    }
  }
  return centroid;
}

Point polygon_normal(const std::vector<Point> &points, const bool &ccw){
  Point normal = construct_utils::make_point();
  if (points.size() >= 3) {
    // selection order on handedness of coordinate system
    auto p0 = ccw ? points[0] : points[2];
    auto p1 = points[1];
    auto p2 = ccw ? points[2] : points[0];

    Point e0 = vec_subt(p1, p0);
    Point e1 = vec_subt(p2, p0);

    // Cross product to get normal vector
    normal.x = e0.y * e1.z - e0.z * e1.y;
    normal.y = e0.z * e1.x - e0.x * e1.z;
    normal.z = e0.x * e1.y - e0.y * e1.x;

    // Normalize the normal vector
    double norm = point_magnitude(normal);
    if (norm != 0) {
      norm = 1 / norm;
      normal = scalar_mult(normal, norm);
    }
  }
  return normal;
}

Quaternion polygon_normal_q(const std::vector<Point> &points, const bool &ccw){
    Point n = polygon_normal(points,ccw);
    Quaternion q;
    // Calculate the norm of the normal vector
    double norm = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);

    // Calculate the angle with respect to the z-axis
    double angle = std::acos(n.z / norm);

    // Calculate sine and cosine of half the angle
    double sin_half_angle = std::sin(angle / 2.0);
    double cos_half_angle = std::cos(angle / 2.0);

    // Calculate quaternion components
    q.w = cos_half_angle;
    q.x = -n.y * sin_half_angle / norm;
    q.y = n.x * sin_half_angle / norm;
    q.z = 0.0; // Assuming rotation around the z-axis

    return q;
}

void normalise(Quaternion &q){
  const double norm = std::sqrt(std::pow(q.w,2)  + std::pow(q.x,2) + std::pow(q.y,2) +std::pow(q.z,2));
  const double iLen = norm !=0 ? 1 / norm : 1;
  q.w *= iLen;
	q.x *= iLen;
	q.y *= iLen;
	q.z *= iLen;
}  

Pose average_pose(const std::vector<Pose> &poses){
  size_t length = poses.size();
  Pose avg = construct_utils::make_pose(0,0,0);
   if (length > 0) {
    for (size_t i=0; i<length; i++) {
       double weight = i > 0 && dot(poses[0].orientation,poses[i].orientation) < 0.0 ? -1 : 1;
       avg.position.x += poses[i].position.x / length;
       avg.position.y += poses[i].position.y / length;
       avg.position.z += poses[i].position.z / length;
       avg.orientation.w += poses[i].orientation.w * weight;
       avg.orientation.x += poses[i].orientation.x * weight;
       avg.orientation.y += poses[i].orientation.y * weight;
       avg.orientation.z += poses[i].orientation.z * weight;
    }
    normalise(avg.orientation);
  }
  return avg;
  
}

} // namespace msg_operators
