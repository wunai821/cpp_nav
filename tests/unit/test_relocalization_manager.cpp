#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/relocalization_manager.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y, float yaw,
                              rm_nav::common::TimePoint stamp) {
  rm_nav::data::Pose3f pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.stamp = stamp;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.is_valid = true;
  return pose;
}

rm_nav::localization::StaticMap MakeTwinMap() {
  rm_nav::localization::StaticMap map;
  const float spacing = 0.15F;
  for (float center_x : {0.0F, 2.0F}) {
    for (int x = -2; x <= 2; ++x) {
      for (int y = -2; y <= 2; ++y) {
        rm_nav::data::PointXYZI point;
        point.x = center_x + static_cast<float>(x) * spacing;
        point.y = static_cast<float>(y) * spacing;
        map.global_points.push_back(point);
      }
    }
  }
  map.global_map_loaded = true;
  return map;
}

rm_nav::data::LidarFrame MakeScanFromCluster(float cluster_center_x,
                                             rm_nav::common::TimePoint stamp) {
  rm_nav::data::LidarFrame scan;
  scan.stamp = stamp;
  scan.scan_begin_stamp = stamp - std::chrono::milliseconds(100);
  scan.scan_end_stamp = stamp;
  const float spacing = 0.15F;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      rm_nav::data::PointXYZI point;
      point.x = cluster_center_x + static_cast<float>(x) * spacing - cluster_center_x;
      point.y = static_cast<float>(y) * spacing;
      point.relative_time_s =
          static_cast<float>((x + 2) * 5 + (y + 2)) * 0.001F;
      scan.points.push_back(point);
    }
  }
  return scan;
}

}  // namespace

int main() {
  rm_nav::config::LocalizationConfig config;
  config.relocalization_retry_interval_ms = 0;
  config.relocalization_submap_radius_m = 0.8;
  config.relocalization_linear_search_step_m = 1.0;
  config.relocalization_yaw_search_step_rad = 0.2;
  config.relocalization_min_match_score = 0.2;
  config.relocalization_stabilization_frames = 2;
  config.relocalization_stabilization_time_ms = 0;
  config.relocalization_ambiguity_score_gap = 1.0;
  config.relocalization_ambiguity_translation_m = 0.0;
  config.relocalization_ambiguity_yaw_rad = 0.0;
  config.relocalization_recovery_spin_wz_radps = 0.4;
  config.relocalization_recovery_backoff_vx_mps = 0.15;
  config.relocalization_recovery_backoff_ticks = 3;

  rm_nav::localization::RelocalizationManager manager;
  assert(manager.Configure(config).ok());

  const auto map = MakeTwinMap();
  const auto first_stamp = rm_nav::common::Now();
  const auto scan = MakeScanFromCluster(0.0F, first_stamp);
  const auto between_pose = MakePose(1.0F, 0.0F, 0.0F, first_stamp);

  manager.SetLastTrustedPose(between_pose);
  manager.UpdateLockState(first_stamp, true);

  rm_nav::localization::ScanMatchResult match;
  rm_nav::localization::RelocalizationStatus status;
  assert(manager
             .Attempt(map, scan, between_pose, between_pose, between_pose, first_stamp, &match,
                      &status)
             .ok());
  assert(status.active);
  assert(status.attempted);
  assert(!status.succeeded);
  assert(status.ambiguity_rejected);
  assert(status.secondary_check_performed);
  assert(!status.secondary_check_passed);
  assert(status.recovery_action ==
         rm_nav::localization::RelocalizationRecoveryAction::kSlowSpin);
  assert(status.recovery_cmd.wz_radps > 0.0F);
  assert(!status.recovery_cmd.brake);

  const auto second_stamp = first_stamp + std::chrono::milliseconds(120);
  assert(manager
             .Attempt(map, scan, between_pose, between_pose, between_pose, second_stamp, &match,
                      &status)
             .ok());
  assert(status.failed_attempts >= 2);
  assert(status.recovery_action ==
         rm_nav::localization::RelocalizationRecoveryAction::kBackoffSpin);
  assert(status.recovery_cmd.vx_mps < 0.0F);
  assert(!status.recovery_cmd.brake);

  const auto third_stamp = second_stamp + std::chrono::milliseconds(120);
  assert(manager
             .Attempt(map, scan, between_pose, between_pose, between_pose, third_stamp, &match,
                      &status)
             .ok());
  const auto fourth_stamp = third_stamp + std::chrono::milliseconds(120);
  assert(manager
             .Attempt(map, scan, between_pose, between_pose, between_pose, fourth_stamp, &match,
                      &status)
             .ok());
  assert(status.failed_attempts >= 4);
  assert(status.recovery_action ==
         rm_nav::localization::RelocalizationRecoveryAction::kFixedPoseSweep);
  assert(status.recovery_cmd.wz_radps > 0.0F);
  assert(!status.recovery_cmd.brake);

  rm_nav::config::LocalizationConfig success_config = config;
  success_config.relocalization_ambiguity_score_gap = 0.0;
  success_config.relocalization_ambiguity_translation_m = 10.0;
  success_config.relocalization_ambiguity_yaw_rad = 10.0;

  rm_nav::localization::RelocalizationManager success_manager;
  assert(success_manager.Configure(success_config).ok());
  success_manager.UpdateLockState(first_stamp, true);

  const auto true_pose = MakePose(0.0F, 0.0F, 0.0F, first_stamp);
  assert(success_manager
             .Attempt(map, scan, true_pose, true_pose, true_pose, first_stamp, &match, &status)
             .ok());
  assert(status.succeeded);
  assert(status.phase == rm_nav::localization::RelocalizationPhase::kStabilizing);
  assert(status.active);

  success_manager.RecordStabilizationObservation(first_stamp + std::chrono::milliseconds(120),
                                                 true_pose, true, &status);
  assert(status.phase == rm_nav::localization::RelocalizationPhase::kStabilizing);
  assert(status.active);
  success_manager.RecordStabilizationObservation(first_stamp + std::chrono::milliseconds(240),
                                                 true_pose, true, &status);
  assert(status.phase == rm_nav::localization::RelocalizationPhase::kTracking);
  assert(!status.active);

  rm_nav::localization::RelocalizationManager failure_manager;
  assert(failure_manager.Configure(success_config).ok());
  failure_manager.UpdateLockState(first_stamp, true);
  assert(failure_manager
             .Attempt(map, scan, true_pose, true_pose, true_pose, first_stamp, &match, &status)
             .ok());
  assert(status.succeeded);
  failure_manager.SetPendingMapToOdomTarget(MakePose(0.6F, -0.2F, 0.1F, first_stamp));
  assert(failure_manager.has_pending_map_to_odom_target());
  failure_manager.RecordStabilizationObservation(first_stamp + std::chrono::milliseconds(120),
                                                 MakePose(3.0F, 3.0F, 1.5F, first_stamp), true,
                                                 &status);
  assert(status.phase == rm_nav::localization::RelocalizationPhase::kSearching);
  assert(status.stabilization_failed);
  assert(!failure_manager.has_pending_map_to_odom_target());

  std::cout << "test_relocalization_manager passed\n";
  return 0;
}
