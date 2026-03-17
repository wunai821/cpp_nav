#pragma once

#include <array>
#include <cstddef>

#include "rm_nav/data/point_types.hpp"

namespace rm_nav::data {

struct PointBlock {
  static constexpr std::size_t kCapacity = 256;

  std::array<PointXYZI, kCapacity> points{};
  std::size_t count{0};
};

}  // namespace rm_nav::data
