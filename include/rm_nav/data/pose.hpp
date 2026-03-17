#pragma once

#include <string_view>

#include "rm_nav/common/time.hpp"
#include "rm_nav/common/types.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::data {

struct Pose3f {
  common::TimePoint stamp{};
  std::string_view reference_frame{tf::kMapFrame};
  std::string_view child_frame{tf::kBaseLinkFrame};
  common::Vec3f position{};
  common::Vec3f rpy{};
  bool is_valid{false};
};

}  // namespace rm_nav::data
