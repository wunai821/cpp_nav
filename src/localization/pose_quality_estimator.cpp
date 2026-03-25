#include "rm_nav/localization/pose_quality_estimator.hpp"

#include <cmath>

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

}  // namespace

LocalizationStatus PoseQualityEstimator::Evaluate(const data::Pose3f& previous_pose,
                                                  const ScanMatchResult& match,
                                                  std::uint32_t consecutive_failures,
                                                  bool map_loaded,
                                                  bool allow_large_jump) const {
  LocalizationStatus status;
  status.match_score = match.score;
  status.iterations = match.iterations;
  status.converged = match.converged;
  status.consecutive_failures = consecutive_failures;
  status.map_loaded = map_loaded;

  if (previous_pose.is_valid && match.matched_pose.is_valid) {
    const float dx = match.matched_pose.position.x - previous_pose.position.x;
    const float dy = match.matched_pose.position.y - previous_pose.position.y;
    status.pose_jump_m = std::sqrt(dx * dx + dy * dy);
    status.yaw_jump_rad =
        std::fabs(NormalizeAngle(match.matched_pose.rpy.z - previous_pose.rpy.z));
  }

  status.pose_trusted =
      map_loaded && match.converged &&
      match.score >= static_cast<float>(config_.min_match_score) &&
      (allow_large_jump ||
       (status.pose_jump_m <= static_cast<float>(config_.max_position_jump_m) &&
        status.yaw_jump_rad <= static_cast<float>(config_.max_yaw_jump_rad))) &&
      consecutive_failures == 0U;
  if (!map_loaded) {
    status.rejection_reason = "map_unloaded";
    status.degraded_mode = "map_missing";
  } else if (!match.converged) {
    status.rejection_reason = "matcher_not_converged";
    status.degraded_mode = "scan_match_failed";
  } else if (match.score < static_cast<float>(config_.min_match_score)) {
    status.rejection_reason = "low_match_score";
    status.degraded_mode = "scan_match_failed";
  } else if (!allow_large_jump &&
             (status.pose_jump_m > static_cast<float>(config_.max_position_jump_m) ||
              status.yaw_jump_rad > static_cast<float>(config_.max_yaw_jump_rad))) {
    status.rejection_reason = "pose_jump_guard";
    status.degraded_mode = "jump_guard";
  } else if (consecutive_failures != 0U) {
    status.rejection_reason = "consecutive_failures";
    status.degraded_mode = "recovery_pending";
  } else {
    status.rejection_reason = "none";
    status.degraded_mode = "none";
  }
  status.rejected_because = status.rejection_reason;
  return status;
}

}  // namespace rm_nav::localization
