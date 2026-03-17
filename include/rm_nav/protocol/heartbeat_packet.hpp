#pragma once

#include <cstdint>

namespace rm_nav::protocol {

struct HeartbeatPacket {
  std::uint64_t monotonic_ns{0};
  std::uint8_t state{0};
};

}  // namespace rm_nav::protocol
