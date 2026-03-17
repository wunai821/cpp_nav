#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"

namespace {

bool DifferentEnough(float lhs, float rhs, float epsilon = 1e-4F) {
  return std::fabs(lhs - rhs) > epsilon;
}

}  // namespace

int main() {
  rm_nav::sync::SensorSync sync;
  rm_nav::sync::SyncConfig config;
  config.max_imu_packets_per_frame = 16;
  config.imu_slice_margin_ns = 0;
  assert(sync.Configure(config).ok());

  for (int index = 0; index <= 10; ++index) {
    rm_nav::data::ImuPacket packet;
    packet.stamp = rm_nav::common::FromNanoseconds(100000000 + index * 10000000);
    packet.sample_index = static_cast<std::uint32_t>(index);
    packet.angular_velocity.z = 1.0F;
    packet.linear_acceleration.x = 0.5F;
    packet.is_valid = true;
    assert(sync.PushImuPacket(packet).ok());
  }

  rm_nav::data::LidarFrame lidar;
  lidar.scan_begin_stamp = rm_nav::common::FromNanoseconds(100000000);
  lidar.scan_end_stamp = rm_nav::common::FromNanoseconds(200000000);
  lidar.stamp = lidar.scan_end_stamp;
  lidar.frame_index = 1;
  rm_nav::data::PointXYZI first;
  first.x = 1.0F;
  first.y = 0.0F;
  first.relative_time_s = 0.0F;
  rm_nav::data::PointXYZI second;
  second.x = 1.0F;
  second.y = 1.0F;
  second.relative_time_s = 0.1F;
  lidar.points.push_back(first);
  lidar.points.push_back(second);
  assert(sync.PushLidarFrame(lidar).ok());
  assert(sync.ProcessOnce().ok());

  auto synced = sync.TryPopSyncedFrame();
  assert(synced.has_value());
  assert(synced->stamp == lidar.scan_end_stamp);
  assert(synced->imu_packet_count > 0U);
  assert(synced->preint.is_valid);
  assert(synced->lidar.is_deskewed);
  assert(synced->sync_latency_ns >= 0);
  assert(DifferentEnough(synced->lidar.points[1].x, second.x) ||
         DifferentEnough(synced->lidar.points[1].y, second.y));

  std::cout << "test_sensor_sync passed\n";
  return 0;
}
