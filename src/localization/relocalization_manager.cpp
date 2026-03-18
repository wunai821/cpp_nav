#include "rm_nav/localization/relocalization_manager.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::localization {
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

data::Pose3f OffsetPose(const data::Pose3f& pose, float dx, float dy, float dyaw) {
  data::Pose3f shifted = pose;
  shifted.position.x += dx;
  shifted.position.y += dy;
  shifted.rpy.z = NormalizeAngle(shifted.rpy.z + dyaw);
  shifted.reference_frame = tf::kMapFrame;
  shifted.child_frame = tf::kBaseLinkFrame;
  shifted.is_valid = true;
  return shifted;
}

}  // namespace

common::Status RelocalizationManager::Configure(const config::LocalizationConfig& config) {
  config_ = config;

  ScanMatchConfig coarse_config;
  coarse_config.max_iterations =
      std::max(1, config_.relocalization_max_iterations);
  coarse_config.correspondence_distance_m =
      std::max(0.2F, static_cast<float>(config_.correspondence_distance_m * 1.5));
  coarse_config.min_match_score =
      static_cast<float>(std::max(0.1, config_.relocalization_min_match_score));

  auto status = coarse_icp_matcher_.Configure(coarse_config);
  if (!status.ok()) {
    return status;
  }
  status = coarse_ndt_matcher_.Configure(coarse_config);
  if (!status.ok()) {
    return status;
  }
  last_attempt_stamp_ = {};
  configured_ = true;
  return common::Status::Ok();
}

StaticMap RelocalizationManager::BuildLocalSubmap(const StaticMap& map,
                                                  const data::Pose3f& center_pose) const {
  StaticMap submap = map;
  submap.global_points.clear();
  const float radius_m = static_cast<float>(std::max(0.5, config_.relocalization_submap_radius_m));
  const float radius_sq = radius_m * radius_m;
  for (const auto& point : map.global_points) {
    const float dx = point.x - center_pose.position.x;
    const float dy = point.y - center_pose.position.y;
    if (dx * dx + dy * dy <= radius_sq) {
      submap.global_points.push_back(point);
    }
  }

  const std::size_t max_points =
      static_cast<std::size_t>(std::max(1, config_.relocalization_submap_max_points));
  if (submap.global_points.size() > max_points) {
    std::vector<data::PointXYZI> downsampled;
    downsampled.reserve(max_points);
    for (std::size_t index = 0; index < max_points; ++index) {
      const std::size_t source_index = (index * submap.global_points.size()) / max_points;
      downsampled.push_back(submap.global_points[source_index]);
    }
    submap.global_points = std::move(downsampled);
  }
  return submap;
}

std::vector<data::Pose3f> RelocalizationManager::BuildCandidates(
    const data::Pose3f& predicted_guess, const data::Pose3f& fallback_guess) const {
  const float step_m =
      static_cast<float>(std::max(0.2, config_.relocalization_linear_search_step_m));
  const float yaw_step =
      static_cast<float>(std::max(0.05, config_.relocalization_yaw_search_step_rad));
  const std::array<std::tuple<float, float, float>, 11> offsets = {{
      {0.0F, 0.0F, 0.0F},
      {step_m, 0.0F, 0.0F},
      {-step_m, 0.0F, 0.0F},
      {0.0F, step_m, 0.0F},
      {0.0F, -step_m, 0.0F},
      {step_m, step_m, 0.0F},
      {-step_m, step_m, 0.0F},
      {step_m, -step_m, 0.0F},
      {-step_m, -step_m, 0.0F},
      {0.0F, 0.0F, yaw_step},
      {0.0F, 0.0F, -yaw_step},
  }};

  std::vector<data::Pose3f> candidates;
  candidates.reserve(offsets.size() * 2U);
  const auto append_seed = [&](const data::Pose3f& seed) {
    if (!seed.is_valid) {
      return;
    }
    for (const auto& [dx, dy, dyaw] : offsets) {
      candidates.push_back(OffsetPose(seed, dx, dy, dyaw));
    }
  };
  append_seed(predicted_guess);
  append_seed(fallback_guess);
  return candidates;
}

common::Status RelocalizationManager::Attempt(const StaticMap& map, const data::LidarFrame& scan,
                                              const data::Pose3f& predicted_guess,
                                              const data::Pose3f& fallback_guess,
                                              common::TimePoint stamp,
                                              ScanMatchResult* best_match,
                                              RelocalizationStatus* status) {
  if (best_match == nullptr || status == nullptr) {
    return common::Status::InvalidArgument("relocalization outputs must not be null");
  }
  if (!configured_) {
    return common::Status::NotReady("relocalization manager is not configured");
  }

  *best_match = {};
  *status = {};
  status->active = true;
  if (!map.global_map_loaded || map.global_points.empty() || scan.points.empty()) {
    status->matcher_used = "none";
    return common::Status::NotReady("relocalization needs map and scan points");
  }

  if (last_attempt_stamp_ != common::TimePoint{} &&
      std::chrono::duration_cast<std::chrono::milliseconds>(stamp - last_attempt_stamp_).count() <
          config_.relocalization_retry_interval_ms) {
    status->matcher_used = "cooldown";
    return common::Status::Ok();
  }

  last_attempt_stamp_ = stamp;
  status->attempted = true;

  const auto begin_ns = common::NowNs();
  const auto candidates = BuildCandidates(predicted_guess, fallback_guess);
  ScanMatchResult best_candidate_match;
  float best_score = -1.0F;

  for (const auto& candidate : candidates) {
    auto local_submap = BuildLocalSubmap(map, candidate);
    if (local_submap.global_points.empty()) {
      continue;
    }

    ScanMatchResult candidate_match;
    auto match_status = coarse_ndt_matcher_.Match(local_submap, scan, candidate, &candidate_match);
    bool fallback_used = false;
    std::string_view matcher_used = "ndt";
    if (!match_status.ok() && match_status.code == common::StatusCode::kUnimplemented) {
      fallback_used = true;
      matcher_used = "icp";
      match_status = coarse_icp_matcher_.Match(local_submap, scan, candidate, &candidate_match);
    }
    if (!match_status.ok()) {
      continue;
    }

    ++status->candidates_tested;
    if (candidate_match.score > best_score) {
      best_score = candidate_match.score;
      best_candidate_match = candidate_match;
      status->best_initial_guess = candidate;
      status->matched_pose = candidate_match.matched_pose;
      status->best_score = candidate_match.score;
      status->fallback_matcher_used = fallback_used;
      status->matcher_used = matcher_used;
      status->succeeded = candidate_match.converged;
    }
  }

  status->attempt_latency_ns = common::NowNs() - begin_ns;
  if (best_score < 0.0F) {
    status->matcher_used = "none";
    return common::Status::Ok();
  }

  *best_match = best_candidate_match;
  status->succeeded = best_candidate_match.converged;
  return common::Status::Ok();
}

}  // namespace rm_nav::localization
