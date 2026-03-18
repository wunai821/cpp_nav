#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp, float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

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

std::vector<rm_nav::data::PointXYZI> MakeWorldPoints() {
  std::vector<rm_nav::data::PointXYZI> points;
  for (int index = 0; index < 36; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 1.0F + 0.06F * static_cast<float>(index % 6);
    point.y = -0.9F + 0.12F * static_cast<float>(index / 6);
    point.z = 0.6F;
    point.intensity = 1.0F;
    points.push_back(point);
  }
  return points;
}

rm_nav::data::SyncedFrame MakeFrame(const std::vector<rm_nav::data::PointXYZI>& world_points,
                                    rm_nav::common::TimePoint stamp, std::uint32_t frame_index,
                                    float x, float y, float yaw) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = stamp;
  frame.lidar.stamp = stamp;
  frame.lidar.scan_begin_stamp = stamp;
  frame.lidar.scan_end_stamp = stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.frame_id = rm_nav::tf::kLaserFrame;
  frame.lidar.is_deskewed = true;
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    frame.lidar.points.push_back(
        ToLocal(world_points[index], x, y, yaw, static_cast<float>(index) * 0.001F));
  }
  return frame;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.voxel_size_m = 0.1;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.occupancy_resolution_m = 0.1;
  config.keyframe_translation_threshold_m = 0.4;
  config.keyframe_yaw_threshold_rad = 0.2;
  config.max_keyframes = 8;
  config.keyframe_max_points = 64;
  config.loop_candidate_distance_threshold_m = 0.4;
  config.loop_candidate_min_time_separation_s = 5.0;
  config.loop_candidate_max_yaw_delta_rad = 0.4;
  config.loop_matcher = "icp";
  config.loop_match_max_iterations = 12;
  config.loop_match_correspondence_distance_m = 0.8;
  config.loop_match_min_score = 0.2;
  config.loop_match_max_points = 64;

  rm_nav::mapping::MappingEngine engine;
  assert(engine.Initialize(config).ok());

  const auto world_points = MakeWorldPoints();
  const auto t0 = rm_nav::common::Now();
  const auto pose0 = MakePose(t0, 0.0F, 0.0F, 0.0F);
  const auto pose1 = MakePose(t0 + std::chrono::seconds(1), 1.0F, 0.0F, 0.0F);
  const auto pose2 = MakePose(t0 + std::chrono::seconds(7), 0.05F, 0.02F, 0.04F);

  assert(engine.Update(MakeFrame(world_points, t0, 1U, 0.0F, 0.0F, 0.0F), pose0).ok());
  assert(engine.Update(MakeFrame(world_points, t0 + std::chrono::seconds(1), 2U, 1.0F, 0.0F, 0.0F),
                       pose1)
             .ok());
  assert(engine.Update(MakeFrame(world_points, t0 + std::chrono::seconds(7), 3U, 0.05F, 0.02F, 0.04F),
                       pose2)
             .ok());

  const auto candidate = engine.LatestLoopCandidate();
  const auto match = engine.LatestLoopMatch();
  const auto correction = engine.LatestLoopCorrection();
  assert(candidate.found);
  assert(candidate.frame_index == 1U);
  assert(match.attempted);
  assert(match.candidate_found);
  assert(match.status_code == rm_nav::common::StatusCode::kOk);
  assert(match.converged);
  assert(match.score >= 0.2F);
  assert(match.translation_correction_m < 0.25F);
  assert(match.yaw_correction_rad < 0.2F);
  assert(correction.evaluated);
  assert(correction.accepted);
  assert(correction.reason == rm_nav::mapping::LoopCorrectionDecisionReason::kAccepted);
  assert(correction.consecutive_failures_after == 0);
  assert(correction.translation_correction_m == match.translation_correction_m);
  assert(correction.yaw_correction_rad == match.yaw_correction_rad);

  std::cout << "test_mapping_loop_match passed\n";
  return 0;
}
