#pragma once

#include <cstdint>

#include "rm_nav/common/time.hpp"

namespace rm_nav::data {

struct OdomState {
  common::TimePoint stamp{};
  std::uint32_t sequence{0};
  float x_m{0.0F};
  float y_m{0.0F};
  float yaw_rad{0.0F};
  float vx_mps{0.0F};
  float vy_mps{0.0F};
  float wz_radps{0.0F};
};

}  // namespace rm_nav::data
