#include "rm_nav/drivers/imu/imu_driver.hpp"

#include <cmath>

#include "rm_nav/common/time.hpp"

namespace rm_nav::drivers::imu {

common::Status ImuDriver::Configure(const ImuDriverConfig& config) {
  if (config.source != "synthetic") {
    return common::Status::Unimplemented("only synthetic IMU source is available");
  }
  if (config.sample_rate_hz <= 0.0) {
    return common::Status::InvalidArgument("invalid synthetic IMU config");
  }
  const auto period = std::chrono::duration_cast<common::Duration>(
      std::chrono::duration<double>(1.0 / config.sample_rate_hz));
  config_ = config;
  next_sample_time_ = common::Now() + period;
  sample_index_ = 0;
  configured_ = true;
  return common::Status::Ok();
}

std::optional<data::ImuPacket> ImuDriver::PollPacket() {
  if (!configured_) {
    return std::nullopt;
  }

  const auto period = std::chrono::duration_cast<common::Duration>(
      std::chrono::duration<double>(1.0 / config_.sample_rate_hz));
  const auto now = common::Now();
  if (next_sample_time_ > now) {
    common::SleepUntil(next_sample_time_);
  }

  const float phase = static_cast<float>(sample_index_) * 0.05F;
  const float noise = config_.noise_stddev * std::sin(phase);

  data::ImuPacket packet;
  packet.stamp = common::Now();
  packet.sample_index = sample_index_++;
  packet.angular_velocity.x = noise;
  packet.angular_velocity.y = -noise;
  packet.angular_velocity.z = config_.gyro_bias_z + noise * 0.5F;
  packet.linear_acceleration.x = noise * 0.2F;
  packet.linear_acceleration.y = -noise * 0.2F;
  packet.linear_acceleration.z = 9.81F + config_.accel_bias_z + noise;
  packet.is_valid = true;

  next_sample_time_ += period;
  if (next_sample_time_ < packet.stamp) {
    next_sample_time_ = packet.stamp + period;
  }
  return packet;
}

}  // namespace rm_nav::drivers::imu
