#pragma once

#include <cstdint>
#include <string_view>

#include "rm_nav/common/time.hpp"
#include "rm_nav/common/types.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::data {

struct ImuPacket {
  common::TimePoint stamp{};
  std::string_view frame_id{tf::kImuFrame};
  std::uint32_t sample_index{0};
  common::Vec3f angular_velocity{};
  common::Vec3f linear_acceleration{};
  bool is_valid{true};
};

}  // namespace rm_nav::data
