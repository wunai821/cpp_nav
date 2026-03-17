#pragma once

#include <cstddef>

#include "rm_nav/common/time.hpp"
#include "rm_nav/common/types.hpp"

namespace rm_nav::data {

struct PreintegratedImuBlock {
  common::TimePoint begin_stamp{};
  common::TimePoint end_stamp{};
  common::Duration duration{};
  common::Vec3f delta_rpy{};
  common::Vec3f delta_velocity{};
  common::Vec3f delta_position{};
  std::size_t sample_count{0};
  common::TimeNs integration_latency_ns{0};
  bool is_valid{false};
};

}  // namespace rm_nav::data
