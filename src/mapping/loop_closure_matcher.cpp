#include "rm_nav/mapping/loop_closure_matcher.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/common/time.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::mapping {
namespace {

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

data::Pose3f RelativePoseInKeyframeFrame(const data::Pose3f& reference,
                                         const data::Pose3f& current) {
  data::Pose3f relative;
  relative.stamp = current.stamp;
  relative.reference_frame = tf::kBaseLinkFrame;
  relative.child_frame = tf::kBaseLinkFrame;
  relative.is_valid = reference.is_valid && current.is_valid;
  if (!relative.is_valid) {
    return relative;
  }

  const float dx = current.position.x - reference.position.x;
  const float dy = current.position.y - reference.position.y;
  const float cos_yaw = std::cos(reference.rpy.z);
  const float sin_yaw = std::sin(reference.rpy.z);
  relative.position.x = cos_yaw * dx + sin_yaw * dy;
  relative.position.y = -sin_yaw * dx + cos_yaw * dy;
  relative.position.z = current.position.z - reference.position.z;
  relative.rpy.x = current.rpy.x - reference.rpy.x;
  relative.rpy.y = current.rpy.y - reference.rpy.y;
  relative.rpy.z = NormalizeAngle(current.rpy.z - reference.rpy.z);
  return relative;
}

data::LidarFrame DownsampleScan(const data::LidarFrame& frame, int max_points) {
  data::LidarFrame downsampled = frame;
  const std::size_t point_limit = static_cast<std::size_t>(std::max(1, max_points));
  if (frame.points.size() <= point_limit) {
    return downsampled;
  }

  downsampled.points.clear();
  downsampled.points.reserve(point_limit);
  for (std::size_t index = 0; index < point_limit; ++index) {
    const std::size_t source_index = (index * frame.points.size()) / point_limit;
    downsampled.points.push_back(frame.points[source_index]);
  }
  return downsampled;
}

}  // namespace

common::Status LoopClosureMatcher::Configure(const config::MappingConfig& config) {
  matcher_ = config.loop_matcher;
  scan_match_config_.max_iterations = config.loop_match_max_iterations;
  scan_match_config_.correspondence_distance_m =
      static_cast<float>(config.loop_match_correspondence_distance_m);
  scan_match_config_.min_match_score = static_cast<float>(config.loop_match_min_score);
  max_points_ = std::max(1, config.loop_match_max_points);

  auto status = icp_matcher_.Configure(scan_match_config_);
  if (!status.ok()) {
    return status;
  }
  status = ndt_matcher_.Configure(scan_match_config_);
  if (!status.ok()) {
    return status;
  }
  if (matcher_ != "icp" && matcher_ != "ndt") {
    return common::Status::InvalidArgument("loop matcher must be icp or ndt");
  }
  return common::Status::Ok();
}

common::Status LoopClosureMatcher::Match(const data::SyncedFrame& frame,
                                         const data::Pose3f& current_map_to_base,
                                         const MappingKeyframe& keyframe,
                                         const LoopClosureCandidate& candidate,
                                         LoopClosureMatchResult* result) const {
  if (result == nullptr) {
    return common::Status::InvalidArgument("loop match result output must not be null");
  }

  *result = {};
  result->attempted = true;
  result->candidate_found = candidate.found;
  result->keyframe_index = candidate.keyframe_index;
  result->frame_index = candidate.frame_index;

  if (!candidate.found) {
    result->status_message = "no loop candidate";
    return common::Status::Ok();
  }
  if (keyframe.local_points.empty() || frame.lidar.points.empty()) {
    result->status_message = "loop match needs scan points and keyframe points";
    return common::Status::Ok();
  }

  const auto begin_ns = common::NowNs();
  localization::StaticMap local_map;
  local_map.global_points = keyframe.local_points;
  local_map.global_map_loaded = true;

  const data::LidarFrame scan = DownsampleScan(frame.lidar, max_points_);
  const data::Pose3f initial_guess =
      RelativePoseInKeyframeFrame(keyframe.map_to_base, current_map_to_base);
  result->initial_guess_candidate_frame = initial_guess;

  localization::ScanMatchResult match;
  common::Status status;
  if (matcher_ == "ndt") {
    status = ndt_matcher_.Match(local_map, scan, initial_guess, &match);
  } else {
    status = icp_matcher_.Match(local_map, scan, initial_guess, &match);
  }

  result->processing_latency_ns = common::NowNs() - begin_ns;
  result->status_code = status.code;
  result->status_message = status.message;
  if (!status.ok()) {
    return common::Status::Ok();
  }

  result->matched_pose_candidate_frame = match.matched_pose;
  result->score = match.score;
  result->iterations = match.iterations;
  result->converged = match.converged;
  const float dx = match.matched_pose.position.x - initial_guess.position.x;
  const float dy = match.matched_pose.position.y - initial_guess.position.y;
  result->translation_correction_m = std::sqrt(dx * dx + dy * dy);
  result->yaw_correction_rad =
      std::fabs(NormalizeAngle(match.matched_pose.rpy.z - initial_guess.rpy.z));
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
