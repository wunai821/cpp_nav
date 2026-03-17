#pragma once

#include <array>
#include <cstdint>

#include "rm_nav/common/time.hpp"

namespace rm_nav::data {

struct DebugSample {
  common::TimePoint stamp{};
  std::array<char, 32> channel{};
  std::array<float, 8> metrics{};
  std::uint32_t metric_count{0};
};

}  // namespace rm_nav::data
