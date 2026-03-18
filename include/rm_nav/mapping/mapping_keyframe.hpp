#pragma once

#include <cstdint>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/point_types.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::mapping {

struct MappingKeyframe {
  common::TimePoint stamp{};
  std::uint32_t frame_index{0};
  data::Pose3f map_to_base{};
  std::vector<data::PointXYZI> local_points{};
};

}  // namespace rm_nav::mapping
