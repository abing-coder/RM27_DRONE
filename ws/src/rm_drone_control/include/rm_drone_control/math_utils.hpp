#pragma once

#include <cmath>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace rm_drone
{

inline Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

inline Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

inline geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion m;
  m.w = q.w();
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  return m;
}

inline geometry_msgs::msg::Vector3 toMsg(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 m;
  m.x = v.x();
  m.y = v.y();
  m.z = v.z();
  return m;
}

inline double yawOf(const Eigen::Quaterniond & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace rm_drone
