#include <cassert>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/perception/local_costmap_builder.hpp"
#include "rm_nav/perception/mot_manager.hpp"
#include "rm_nav/perception/preprocess_pipeline.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"
#include "rm_nav/tf/transform_query.hpp"

namespace {

void PushSyntheticFrame(rm_nav::sync::SensorSync* sync, std::uint32_t frame_index,
                        rm_nav::common::TimeNs base_stamp_ns) {
  assert(sync != nullptr);

  for (int index = 0; index < 10; ++index) {
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
  for (int index = 0; index < 120; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 0.03F * static_cast<float>(index);
    point.y = 0.01F * static_cast<float>(index % 8);
    point.z = 0.0F;
    point.intensity = static_cast<float>(index);
    point.relative_time_s = static_cast<float>(index) / 1200.0F;
    lidar.points.push_back(point);
  }
  assert(sync->PushLidarFrame(lidar).ok());
  assert(sync->ProcessOnce().ok());
}

}  // namespace

int main() {
  rm_nav::sync::SensorSync sync;
  rm_nav::sync::SyncConfig sync_config;
  sync_config.max_imu_packets_per_frame = 8;
  assert(sync.Configure(sync_config).ok());

  rm_nav::tf::TfTreeLite tf_tree;
  assert(tf_tree.RegisterStaticTransform(
             rm_nav::tf::MakeTransform(rm_nav::tf::kBaseLinkFrame,
                                       rm_nav::tf::kLaserFrame, 0.2F, 0.0F, 0.0F))
             .ok());

  rm_nav::localization::LocalizationEngine localization;
  const rm_nav::config::LocalizationConfig localization_config;
  const rm_nav::config::SpawnConfig spawn_config;
  assert(localization.Initialize("config", localization_config, spawn_config, &tf_tree).ok());

  rm_nav::perception::PreprocessPipeline preprocess;
  assert(preprocess.Configure({2, 8}).ok());

  rm_nav::perception::LocalCostmapBuilder costmap_builder;
  rm_nav::perception::MotManager mot_manager;
  rm_nav::planning::PlannerCoordinator planner;
  const rm_nav::config::PlannerConfig planner_config;
  assert(planner.Initialize(planner_config, localization.static_map()).ok());

  PushSyntheticFrame(&sync, 3U, 1000000);
  auto localization_frame = sync.TryPopSyncedFrameHandle();
  assert(localization_frame.has_value());
  assert(localization.EnqueueFrame(std::move(*localization_frame)).ok());
  assert(localization.ProcessOnce().ok());

  rm_nav::data::Pose3f map_to_base;
  assert(rm_nav::tf::LookupTransform(tf_tree, rm_nav::tf::kMapFrame,
                                     rm_nav::tf::kBaseLinkFrame,
                                     rm_nav::common::FromNanoseconds(1300000),
                                     &map_to_base)
             .ok());
  assert(map_to_base.is_valid);

  PushSyntheticFrame(&sync, 4U, 3000000);
  auto preprocess_frame = sync.TryPopSyncedFrameHandle();
  assert(preprocess_frame.has_value());
  assert(preprocess.EnqueueFrame(std::move(*preprocess_frame)).ok());
  assert(preprocess.ProcessOnce().ok());

  auto filtered_frame = preprocess.TryPopFilteredFrameHandle();
  assert(filtered_frame.has_value());
  assert(!filtered_frame->get()->points.empty());
  assert(filtered_frame->get()->points.size() < 120U);

  auto planning_pose = localization.LatestResult().map_to_base;
  if (!planning_pose.is_valid || planning_pose.position.x < 0.0F ||
      planning_pose.position.x > 12.0F || planning_pose.position.y < 0.0F ||
      planning_pose.position.y > 6.0F) {
    planning_pose.reference_frame = rm_nav::tf::kMapFrame;
    planning_pose.child_frame = rm_nav::tf::kBaseLinkFrame;
    planning_pose.position.x = 1.2F;
    planning_pose.position.y = 2.0F;
    planning_pose.is_valid = true;
  }
  assert(costmap_builder.BuildAndPublish(*filtered_frame->get(),
                                         planning_pose)
             .ok());
  assert(mot_manager.UpdateAndPublish(*filtered_frame->get(),
                                      planning_pose)
             .ok());
  filtered_frame->reset();

  const auto latest_costmap = costmap_builder.LatestCostmap();
  const auto latest_obstacles = mot_manager.LatestObstacles();
  assert(latest_costmap.width == 80U);
  assert(!latest_obstacles.obstacles.empty());

  assert(planner.PlanAndPublish(planning_pose, latest_costmap,
                                latest_obstacles.obstacles)
             .ok());
  const auto latest_path = planner.LatestPath();
  const auto latest_cmd = planner.LatestCmd();
  assert(!latest_path.points.empty());
  assert(!latest_cmd.brake);

  std::cout << "test_pipeline_flow passed\n";
  return 0;
}
