#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/sync/imu_preintegrator.hpp"

namespace {

bool Near(float lhs, float rhs, float epsilon = 1e-3F) {
  return std::fabs(lhs - rhs) <= epsilon;
}

}  // namespace

int main() {
  rm_nav::sync::ImuPreintegrator preintegrator;
  std::array<rm_nav::data::ImuPacket, 11> packets{};

  for (std::size_t index = 0; index < packets.size(); ++index) {
    packets[index].stamp =
        rm_nav::common::FromNanoseconds(static_cast<rm_nav::common::TimeNs>(index) *
                                        10000000);
    packets[index].angular_velocity.z = 1.0F;
    packets[index].linear_acceleration.x = 1.0F;
    packets[index].is_valid = true;
  }

  rm_nav::data::PreintegratedImuBlock block;
  const auto status =
      preintegrator.Integrate(packets.data(), packets.size(), packets.front().stamp,
                              packets.back().stamp, &block);
  assert(status.ok());
  assert(block.is_valid);
  assert(block.sample_count > 0U);
  assert(Near(block.delta_rpy.z, 0.1F, 0.02F));
  assert(Near(block.delta_velocity.x, 0.1F, 0.02F));
  assert(Near(block.delta_position.x, 0.005F, 0.003F));
  assert(std::fabs(block.delta_position.y) < 0.002F);

  std::cout << "test_preintegration passed\n";
  return 0;
}
