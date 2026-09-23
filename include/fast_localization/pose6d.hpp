#pragma once

#include <array>

namespace fast_localization
{

// This was a ROS 1 private message. It never crossed the node boundary, so a
// value type avoids an unnecessary rosidl interface dependency in ROS 2.
struct Pose6D
{
  double offset_time{0.0};
  std::array<double, 3> acc{};
  std::array<double, 3> gyr{};
  std::array<double, 3> vel{};
  std::array<double, 3> pos{};
  std::array<double, 9> rot{};
};

}  // namespace fast_localization
