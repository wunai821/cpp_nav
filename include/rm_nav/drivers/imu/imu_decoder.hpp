#pragma once

#include <cstddef>
#include <cstdint>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/imu_packet.hpp"

namespace rm_nav::drivers::imu {

class ImuDecoder {
 public:
  common::Status Decode(const std::uint8_t* data, std::size_t size,
                        data::ImuPacket* packet) const;
};

}  // namespace rm_nav::drivers::imu
