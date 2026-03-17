#pragma once

#include <cstdint>

namespace rm_nav::data {

struct HealthReport {
  std::uint64_t tx_packets{0};
  std::uint64_t rx_packets{0};
  std::uint64_t crc_errors{0};
  std::uint64_t parse_errors{0};
  std::uint64_t dropped_packets{0};
};

}  // namespace rm_nav::data
