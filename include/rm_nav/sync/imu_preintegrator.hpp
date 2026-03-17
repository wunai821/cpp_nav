#pragma once

#include <cstddef>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/preint_block.hpp"

namespace rm_nav::sync {

class ImuPreintegrator {
 public:
  common::Status Integrate(const data::ImuPacket* packets, std::size_t count,
                           common::TimePoint begin_stamp,
                           common::TimePoint end_stamp,
                           data::PreintegratedImuBlock* block) const;
};

}  // namespace rm_nav::sync
