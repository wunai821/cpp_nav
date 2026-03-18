#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/mapping/loop_closure_matcher.hpp"
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
  for (int index = 0; index < 24; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 1.5F + 0.08F * static_cast<float>(index % 6);
    point.y = -0.7F + 0.18F * static_cast<float>(index / 6);
    point.z = 0.6F;
    point.intensity = 1.0F;
    points.push_back(point);
  }
  return points;
}

rm_nav::mapping::MappingKeyframe MakeKeyframe(const std::vector<rm_nav::data::PointXYZI>& world_points,
                                              rm_nav::common::TimePoint stamp,
                                              std::uint32_t frame_index, float x, float y,
                                              float yaw) {
  rm_nav::mapping::MappingKeyframe keyframe;
  keyframe.stamp = stamp;
  keyframe.frame_index = frame_index;
  keyframe.map_to_base = MakePose(stamp, x, y, yaw);
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    keyframe.local_points.push_back(
        ToLocal(world_points[index], x, y, yaw, static_cast<float>(index) * 0.001F));
  }
  return keyframe;
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
  frame.lidar.is_deskewed = true;
  frame.lidar.frame_id = rm_nav::tf::kLaserFrame;
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    frame.lidar.points.push_back(
        ToLocal(world_points[index], x, y, yaw, static_cast<float>(index) * 0.001F));
  }
  return frame;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.loop_matcher = "icp";
  config.loop_match_max_iterations = 12;
  config.loop_match_correspondence_distance_m = 0.8;
  config.loop_match_min_score = 0.2;
  config.loop_match_max_points = 64;

  const auto now = rm_nav::common::Now();
  const auto world_points = MakeWorldPoints();
  const auto keyframe = MakeKeyframe(world_points, now - std::chrono::seconds(8), 10U, 0.0F, 0.0F,
                                     0.0F);
  const auto frame = MakeFrame(world_points, now, 100U, 0.08F, -0.04F, 0.06F);
  const auto current_pose = MakePose(now, 0.08F, -0.04F, 0.06F);

  rm_nav::mapping::LoopClosureCandidate candidate;
  candidate.found = true;
  candidate.keyframe_index = 0U;
  candidate.frame_index = keyframe.frame_index;
  candidate.stamp = keyframe.stamp;

  rm_nav::mapping::LoopClosureMatcher matcher;
  assert(matcher.Configure(config).ok());

  rm_nav::mapping::LoopClosureMatchResult result;
  assert(matcher.Match(frame, current_pose, keyframe, candidate, &result).ok());
  assert(result.attempted);
  assert(result.candidate_found);
  assert(result.status_code == rm_nav::common::StatusCode::kOk);
  assert(result.converged);
  assert(result.score >= 0.2F);
  assert(result.iterations > 0);
  assert(result.translation_correction_m < 0.25F);
  assert(result.yaw_correction_rad < 0.2F);
  assert(result.matched_pose_candidate_frame.is_valid);

  std::cout << "test_loop_closure_matcher passed\n";
  return 0;
}
