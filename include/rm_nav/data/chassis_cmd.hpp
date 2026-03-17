#pragma once

#include "rm_nav/common/time.hpp"

namespace rm_nav::data {

struct ChassisCmd {
  common::TimePoint stamp{};
  float vx_mps{0.0F};
  float vy_mps{0.0F};
  float wz_radps{0.0F};
  bool brake{false};
};

}  // namespace rm_nav::data
