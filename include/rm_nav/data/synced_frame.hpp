#pragma once

#include <array>
#include <cstddef>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/preint_block.hpp"

namespace rm_nav::data {

struct SyncedFrame {
  static constexpr std::size_t kMaxImuPackets = 128;

  common::TimePoint stamp{};
  LidarFrame lidar{};
  std::array<ImuPacket, kMaxImuPackets> imu_packets{};
  std::size_t imu_packet_count{0};
  PreintegratedImuBlock preint{};
  common::TimeNs deskew_latency_ns{0};
  common::TimeNs sync_latency_ns{0};

  void ClearImuPackets() { imu_packet_count = 0; }
};

}  // namespace rm_nav::data
