#pragma once

#include <cstdint>

#include "rm_nav/common/time.hpp"

namespace rm_nav::data {

struct RefereeState {
  common::TimePoint stamp{};
  std::uint32_t sequence{0};
  bool is_online{false};
  std::uint8_t game_stage{0};
  std::uint16_t robot_hp{0};
  std::uint16_t ammo{0};
  std::uint16_t remaining_time_s{0};
};

}  // namespace rm_nav::data
