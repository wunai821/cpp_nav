#pragma once

#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/common/types.hpp"

namespace rm_nav::data {

struct PathPoint2f {
  common::Vec2f position{};
  float heading_rad{0.0F};
  float target_speed_mps{0.0F};
};

struct Path2D {
  common::TimePoint stamp{};
  std::vector<PathPoint2f> points{};
};

}  // namespace rm_nav::data
