#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/localization/map_odom_fuser.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"

namespace {

rm_nav::data::PointXYZI ToLocal(const rm_nav::data::PointXYZI& world_point, float x, float y,
                                float yaw, float relative_time_s) {
  const float dx = world_point.x - x;
  const float dy = world_point.y - y;
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  rm_nav::data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  local.relative_time_s = relative_time_s;
  return local;
}

rm_nav::data::SyncedFrame MakeFrame(const rm_nav::localization::StaticMap& map,
                                    rm_nav::common::TimePoint stamp, float x, float y,
                                    float yaw) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = stamp;
  frame.lidar.stamp = stamp;
  frame.lidar.scan_begin_stamp = stamp - std::chrono::milliseconds(100);
  frame.lidar.scan_end_stamp = stamp;
  frame.lidar.frame_index = 7U;

  for (std::size_t index = 0; index < map.global_points.size(); ++index) {
    const auto local =
        ToLocal(map.global_points[index], x, y, yaw, static_cast<float>(index) * 0.001F);
    const float range_sq = local.x * local.x + local.y * local.y;
    if (range_sq > 25.0F || local.x < -1.0F) {
      continue;
    }
    frame.lidar.points.push_back(local);
  }
  assert(!frame.lidar.points.empty());
  return frame;
}

float PoseDistance(const rm_nav::data::Pose3f& left, const rm_nav::data::Pose3f& right) {
  const float dx = left.position.x - right.position.x;
  const float dy = left.position.y - right.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

int main() {
  rm_nav::config::LocalizationConfig localization_config;
  localization_config.relocalization_failure_threshold = 1;
  localization_config.relocalization_retry_interval_ms = 0;
  localization_config.relocalization_linear_search_step_m = 1.5;
  localization_config.relocalization_yaw_search_step_rad = 0.35;
  localization_config.relocalization_min_match_score = 0.2;
  localization_config.relocalization_stabilization_frames = 2;
  localization_config.relocalization_stabilization_time_ms = 0;
  localization_config.relocalization_map_to_odom_apply_translation_step_m = 0.18;
  localization_config.relocalization_map_to_odom_apply_yaw_step_rad = 0.08;
  localization_config.max_position_jump_m = 0.4;
  localization_config.max_yaw_jump_rad = 0.25;

  rm_nav::config::SpawnConfig spawn_config;
  spawn_config.x_m = 2.5;
  spawn_config.y_m = 2.2;
  spawn_config.theta_rad = 0.1;

  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;
  assert(loader.Load("config", localization_config, &map).ok());

  rm_nav::tf::TfTreeLite tf_tree;
  rm_nav::localization::LocalizationEngine localization;
  assert(localization.Initialize("config", localization_config, spawn_config, &tf_tree).ok());

  constexpr float kTrueX = 2.8F;
  constexpr float kTrueY = 2.4F;
  constexpr float kTrueYaw = 0.15F;

  const auto first_stamp = rm_nav::common::Now();
  const auto first_frame = MakeFrame(map, first_stamp, kTrueX, kTrueY, kTrueYaw);

  rm_nav::data::OdomState odom;
  odom.stamp = first_stamp;
  odom.x_m = 2.55F;
  odom.y_m = 2.55F;
  odom.yaw_rad = 0.05F;
  localization.SetLatestOdom(odom);

  rm_nav::localization::LocalizationResult first_result;
  assert(localization.Process(first_frame, &first_result).ok());
  assert(first_result.map_to_odom.is_valid);
  assert(first_result.status.iterations > 0);

  const auto second_stamp = first_stamp + std::chrono::milliseconds(150);
  auto second_frame = MakeFrame(map, second_stamp, kTrueX, kTrueY, kTrueYaw);
  second_frame.lidar.frame_index = 8U;
  second_frame.lidar.points.resize(4U);

  odom.stamp = second_stamp;
  odom.x_m = 3.80F;
  odom.y_m = 3.35F;
  odom.yaw_rad = 0.62F;
  localization.SetLatestOdom(odom);

  rm_nav::localization::LocalizationResult second_result;
  assert(localization.Process(second_frame, &second_result).ok());
  assert(second_result.status.consecutive_failures > 0U);
  assert(!second_result.status.pose_trusted);

  const auto third_stamp = second_stamp + std::chrono::milliseconds(150);
  auto third_frame = MakeFrame(map, third_stamp, kTrueX, kTrueY, kTrueYaw);
  third_frame.lidar.frame_index = 9U;
  odom.stamp = third_stamp;
  odom.x_m = 3.80F;
  odom.y_m = 3.35F;
  odom.yaw_rad = 0.62F;
  localization.SetLatestOdom(odom);

  rm_nav::localization::LocalizationResult third_result;
  assert(localization.Process(third_frame, &third_result).ok());
  assert(third_result.relocalization.active);
  assert(third_result.relocalization.attempted);
  assert(third_result.relocalization.succeeded);
  assert(third_result.relocalization.phase ==
         rm_nav::localization::RelocalizationPhase::kStabilizing);
  assert(!third_result.status.pose_trusted);

  rm_nav::localization::MapOdomFuser fuser;
  rm_nav::data::Pose3f direct_map_to_odom;
  assert(fuser.Fuse(third_result.odom_to_base, third_result.relocalization.matched_pose,
                    &direct_map_to_odom)
             .ok());
  const float blended_step =
      PoseDistance(second_result.map_to_odom, third_result.map_to_odom);
  const float direct_step =
      PoseDistance(second_result.map_to_odom, direct_map_to_odom);
  assert(blended_step > 0.0F);
  assert(blended_step < direct_step);

  const auto fourth_stamp = third_stamp + std::chrono::milliseconds(150);
  auto fourth_frame = MakeFrame(map, fourth_stamp, kTrueX, kTrueY, kTrueYaw);
  fourth_frame.lidar.frame_index = 10U;
  fourth_frame.lidar.points.resize(4U);
  odom.stamp = fourth_stamp;
  odom.x_m = 3.82F;
  odom.y_m = 3.33F;
  odom.yaw_rad = 0.60F;
  localization.SetLatestOdom(odom);

  rm_nav::localization::LocalizationResult fourth_result;
  assert(localization.Process(fourth_frame, &fourth_result).ok());
  assert(fourth_result.relocalization.active);
  assert(fourth_result.relocalization.phase ==
         rm_nav::localization::RelocalizationPhase::kSearching);
  assert(fourth_result.relocalization.stabilization_failed);
  assert(!fourth_result.relocalization.map_to_odom_blending_active);
  assert(PoseDistance(fourth_result.map_to_odom, first_result.map_to_odom) < 1.0e-4F);

  std::cout << "test_localization_recovery passed\n";
  return 0;
}
