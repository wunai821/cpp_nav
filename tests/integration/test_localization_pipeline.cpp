#include <cassert>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"
#include "rm_nav/tf/transform_query.hpp"

namespace {

void PushSyntheticFrame(rm_nav::sync::SensorSync* sync, std::uint32_t frame_index,
                        rm_nav::common::TimeNs base_stamp_ns) {
  assert(sync != nullptr);

  for (int index = 0; index < 12; ++index) {
    rm_nav::data::ImuPacket packet;
    packet.stamp = rm_nav::common::FromNanoseconds(base_stamp_ns + index * 20000);
    packet.sample_index = frame_index * 100U + static_cast<std::uint32_t>(index);
    packet.angular_velocity.z = 0.01F;
    packet.linear_acceleration.z = 9.81F;
    assert(sync->PushImuPacket(packet).ok());
  }

  rm_nav::data::LidarFrame lidar;
  lidar.scan_begin_stamp = rm_nav::common::FromNanoseconds(base_stamp_ns + 200000);
  lidar.scan_end_stamp = rm_nav::common::FromNanoseconds(base_stamp_ns + 300000);
  lidar.stamp = lidar.scan_end_stamp;
  lidar.frame_index = frame_index;
  for (int index = 0; index < 240; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 0.02F * static_cast<float>(index);
    point.y = 0.015F * static_cast<float>(index % 12);
    point.z = 0.2F + 0.01F * static_cast<float>(index % 4);
    point.intensity = static_cast<float>(index);
    point.relative_time_s = static_cast<float>(index) / 2400.0F;
    lidar.points.push_back(point);
  }
  assert(sync->PushLidarFrame(lidar).ok());
  assert(sync->ProcessOnce().ok());
}

}  // namespace

int main() {
  rm_nav::sync::SensorSync sync;
  rm_nav::sync::SyncConfig sync_config;
  sync_config.max_imu_packets_per_frame = rm_nav::data::SyncedFrame::kMaxImuPackets;
  assert(sync.Configure(sync_config).ok());

  rm_nav::tf::TfTreeLite tf_tree;
  assert(tf_tree.RegisterStaticTransform(
             rm_nav::tf::MakeTransform(rm_nav::tf::kBaseLinkFrame,
                                       rm_nav::tf::kLaserFrame, 0.2F, 0.0F, 0.0F))
             .ok());

  rm_nav::localization::LocalizationEngine localization;
  rm_nav::config::LocalizationConfig localization_config;
  rm_nav::config::SpawnConfig spawn_config;
  assert(localization.Initialize("config", localization_config, spawn_config, &tf_tree).ok());

  PushSyntheticFrame(&sync, 5U, 1000000);
  auto handle = sync.TryPopSyncedFrameHandle();
  assert(handle.has_value());
  assert(localization.EnqueueFrame(std::move(*handle)).ok());
  assert(localization.ProcessOnce().ok());

  const auto result = localization.LatestResult();
  assert(result.map_to_odom.is_valid);
  assert(result.odom_to_base.is_valid);
  assert(result.map_to_base.is_valid);
  assert(localization.LatestAlignedScan().frame_id == rm_nav::tf::kMapFrame);

  rm_nav::data::Pose3f queried_map_to_base;
  assert(rm_nav::tf::LookupTransform(tf_tree, rm_nav::tf::kMapFrame,
                                     rm_nav::tf::kBaseLinkFrame, result.map_to_base.stamp,
                                     &queried_map_to_base)
             .ok());
  assert(queried_map_to_base.is_valid);

  std::cout << "test_localization_pipeline passed\n";
  return 0;
}
