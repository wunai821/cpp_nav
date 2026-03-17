#pragma once

#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/common/types.hpp"

namespace rm_nav::data {

struct TrajectoryPoint {
  common::Vec2f position{};
  float heading_rad{0.0F};
  float speed_mps{0.0F};
  float relative_time_s{0.0F};
};

struct Trajectory {
  common::TimePoint stamp{};
  std::vector<TrajectoryPoint> points{};
};

}  // namespace rm_nav::data
