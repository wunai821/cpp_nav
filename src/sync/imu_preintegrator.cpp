#include "rm_nav/sync/imu_preintegrator.hpp"

#include <algorithm>

namespace rm_nav::sync {
namespace {

rm_nav::common::Vec3f AddScaled(const rm_nav::common::Vec3f& lhs,
                                const rm_nav::common::Vec3f& rhs, float scale) {
  return {lhs.x + rhs.x * scale, lhs.y + rhs.y * scale, lhs.z + rhs.z * scale};
}

}  // namespace

common::Status ImuPreintegrator::Integrate(const data::ImuPacket* packets,
                                           std::size_t count,
                                           common::TimePoint begin_stamp,
                                           common::TimePoint end_stamp,
                                           data::PreintegratedImuBlock* block) const {
  if (block == nullptr) {
    return common::Status::InvalidArgument("preintegration output is null");
  }
  *block = {};
  block->begin_stamp = begin_stamp;
  block->end_stamp = end_stamp;
  block->duration = end_stamp - begin_stamp;

  if (count == 0U || packets == nullptr || end_stamp <= begin_stamp) {
    return common::Status::NotReady("imu block is empty");
  }

  common::Vec3f delta_rpy{};
  common::Vec3f delta_v{};
  common::Vec3f delta_p{};
  common::Vec3f velocity{};
  common::TimePoint prev_stamp = begin_stamp;
  std::size_t used_samples = 0;

  for (std::size_t index = 0; index < count; ++index) {
    const auto& packet = packets[index];
    const common::TimePoint sample_stamp =
        packet.stamp < begin_stamp ? begin_stamp
                                   : (packet.stamp > end_stamp ? end_stamp : packet.stamp);
    if (sample_stamp <= prev_stamp) {
      continue;
    }

    const float dt =
        static_cast<float>(common::ToNanoseconds(sample_stamp - prev_stamp)) / 1.0e9F;
    delta_rpy = AddScaled(delta_rpy, packet.angular_velocity, dt);
    delta_v = AddScaled(delta_v, packet.linear_acceleration, dt);
    velocity = AddScaled(velocity, packet.linear_acceleration, dt);
    delta_p = AddScaled(delta_p, velocity, dt);
    prev_stamp = sample_stamp;
    ++used_samples;
  }

  if (prev_stamp < end_stamp) {
    const auto& last_packet = packets[count - 1U];
    const float dt =
        static_cast<float>(common::ToNanoseconds(end_stamp - prev_stamp)) / 1.0e9F;
    delta_rpy = AddScaled(delta_rpy, last_packet.angular_velocity, dt);
    delta_v = AddScaled(delta_v, last_packet.linear_acceleration, dt);
    velocity = AddScaled(velocity, last_packet.linear_acceleration, dt);
    delta_p = AddScaled(delta_p, velocity, dt);
  }

  block->delta_rpy = delta_rpy;
  block->delta_velocity = delta_v;
  block->delta_position = delta_p;
  block->sample_count = used_samples;
  block->is_valid = used_samples > 0U;
  return block->is_valid ? common::Status::Ok()
                         : common::Status::NotReady("imu preintegration has no usable samples");
}

}  // namespace rm_nav::sync
