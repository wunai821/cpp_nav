#include "rm_nav/sync/imu_preintegrator.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::sync {
namespace {

rm_nav::common::Vec3f Add(const rm_nav::common::Vec3f& lhs,
                          const rm_nav::common::Vec3f& rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

rm_nav::common::Vec3f Subtract(const rm_nav::common::Vec3f& lhs,
                               const rm_nav::common::Vec3f& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

rm_nav::common::Vec3f Scale(const rm_nav::common::Vec3f& value, float scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

rm_nav::common::Vec3f Average(const rm_nav::common::Vec3f& lhs,
                              const rm_nav::common::Vec3f& rhs) {
  return Scale(Add(lhs, rhs), 0.5F);
}

rm_nav::common::Vec3f RotateByYaw(const rm_nav::common::Vec3f& value, float yaw) {
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  return {cos_yaw * value.x - sin_yaw * value.y,
          sin_yaw * value.x + cos_yaw * value.y,
          value.z};
}

float SecondsBetween(rm_nav::common::TimePoint begin, rm_nav::common::TimePoint end) {
  return static_cast<float>(rm_nav::common::ToNanoseconds(end - begin)) / 1.0e9F;
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
  std::size_t used_samples = 0;
  const auto begin_ns = common::NowNs();

  for (std::size_t index = 0; index < count; ++index) {
    const auto& current_packet = packets[index];
    if (!current_packet.is_valid) {
      continue;
    }

    const auto& next_packet = (index + 1U < count && packets[index + 1U].is_valid)
                                  ? packets[index + 1U]
                                  : current_packet;
    const common::TimePoint segment_begin =
        std::max(begin_stamp, current_packet.stamp);
    const common::TimePoint raw_segment_end =
        next_packet.stamp > current_packet.stamp ? next_packet.stamp : end_stamp;
    const common::TimePoint segment_end =
        std::min(end_stamp, raw_segment_end);
    if (segment_end <= segment_begin) {
      continue;
    }

    const float dt = SecondsBetween(segment_begin, segment_end);
    const common::Vec3f omega =
        Average(current_packet.angular_velocity, next_packet.angular_velocity);
    const common::Vec3f accel_body =
        Average(current_packet.linear_acceleration, next_packet.linear_acceleration);
    const common::Vec3f delta_rpy_step = Scale(omega, dt);
    const common::Vec3f mid_rpy = Add(delta_rpy, Scale(delta_rpy_step, 0.5F));
    const common::Vec3f accel_world = RotateByYaw(accel_body, mid_rpy.z);

    delta_rpy = Add(delta_rpy, delta_rpy_step);
    delta_p = Add(delta_p, Add(Scale(velocity, dt), Scale(accel_world, 0.5F * dt * dt)));
    velocity = Add(velocity, Scale(accel_world, dt));
    delta_v = Add(delta_v, Scale(accel_world, dt));
    ++used_samples;
  }

  block->delta_rpy = delta_rpy;
  block->delta_velocity = delta_v;
  block->delta_position = delta_p;
  block->sample_count = used_samples;
  block->integration_latency_ns = common::NowNs() - begin_ns;
  block->is_valid = used_samples > 0U;
  return block->is_valid ? common::Status::Ok()
                         : common::Status::NotReady("imu preintegration has no usable samples");
}

}  // namespace rm_nav::sync
