#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/point_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::data {

struct LidarFrame {
  common::TimePoint stamp{};
  common::TimePoint scan_begin_stamp{};
  common::TimePoint scan_end_stamp{};
  std::string_view frame_id{tf::kLaserFrame};
  std::uint32_t frame_index{0};
  std::vector<PointXYZI> points{};
  bool is_deskewed{false};
};

}  // namespace rm_nav::data
