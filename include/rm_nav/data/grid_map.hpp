#pragma once

#include <cstdint>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::data {

struct GridMap2D {
  common::TimePoint stamp{};
  float resolution_m{0.05F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  Pose3f origin{};
  std::vector<std::uint8_t> occupancy{};
};

}  // namespace rm_nav::data
