#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rm_nav/common/status.hpp"

namespace rm_nav::drivers::lidar {

struct L1Packet {
  std::uint32_t packet_index{0};
  std::vector<std::uint8_t> bytes{};
};

class L1PacketParser {
 public:
  common::Status Parse(const std::uint8_t* data, std::size_t size,
                       L1Packet* packet) const;
};

}  // namespace rm_nav::drivers::lidar
