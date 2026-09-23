#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_msgs
{
using Imu = msg::Imu;
using PointCloud2 = msg::PointCloud2;
}  // namespace sensor_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using Path = msg::Path;
}  // namespace nav_msgs

namespace geometry_msgs
{
using PoseStamped = msg::PoseStamped;
using Quaternion = msg::Quaternion;
using TransformStamped = msg::TransformStamped;
using Vector3 = msg::Vector3;
}  // namespace geometry_msgs

inline rclcpp::Logger fast_localization_logger()
{
  return rclcpp::get_logger("fast_localization");
}

inline double stamp_to_sec(const builtin_interfaces::msg::Time &stamp)
{
  return rclcpp::Time(stamp).seconds();
}

inline builtin_interfaces::msg::Time stamp_from_nanoseconds(int64_t nanoseconds)
{
  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

  // builtin_interfaces/Time stores the signed seconds and the non-negative
  // nanosecond remainder separately.  Do this conversion directly instead of
  // relying on rclcpp::Time::to_msg(), which is unavailable in Humble's
  // rclcpp version shipped by this server.
  int64_t seconds = nanoseconds / kNanosecondsPerSecond;
  int64_t remainder = nanoseconds % kNanosecondsPerSecond;
  if (remainder < 0) {
    --seconds;
    remainder += kNanosecondsPerSecond;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(remainder);
  return stamp;
}

inline builtin_interfaces::msg::Time stamp_from_sec(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1.0e9));
  return stamp_from_nanoseconds(nanoseconds);
}

#define ROS_INFO(...) RCLCPP_INFO(fast_localization_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(fast_localization_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(fast_localization_logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(fast_localization_logger(), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) RCLCPP_ERROR(fast_localization_logger(), __VA_ARGS__)
#define ROS_ASSERT(condition) assert(condition)
