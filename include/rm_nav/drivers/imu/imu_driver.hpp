#pragma once

#include <cstdint>
#include <string>
#include <optional>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/imu_packet.hpp"

namespace rm_nav::drivers::imu {

struct ImuDriverConfig {
  std::string source{"synthetic"};
  double sample_rate_hz{200.0};
  float gyro_bias_z{0.01F};
  float accel_bias_z{0.02F};
  float noise_stddev{0.005F};
};

class ImuDriver {
 public:
  common::Status Configure(const ImuDriverConfig& config);
  std::optional<data::ImuPacket> PollPacket();

 private:
  ImuDriverConfig config_{};
  common::TimePoint next_sample_time_{};
  std::uint32_t sample_index_{0};
  bool configured_{false};
};

}  // namespace rm_nav::drivers::imu
