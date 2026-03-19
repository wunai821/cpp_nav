#include "rm_nav/localization/relocalization_manager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>

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

float ClampAbs(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
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

float PoseTranslationDistance(const data::Pose3f& left, const data::Pose3f& right) {
  const float dx = left.position.x - right.position.x;
  const float dy = left.position.y - right.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

common::Status RelocalizationManager::Configure(const config::LocalizationConfig& config) {
  config_ = config;

  ScanMatchConfig coarse_config;
  coarse_config.max_iterations = std::max(1, config_.relocalization_max_iterations);
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

  Reset();
  configured_ = true;
  return common::Status::Ok();
}

void RelocalizationManager::Reset() {
  last_attempt_stamp_ = {};
  lost_lock_started_at_ = {};
  stabilization_started_at_ = {};
  last_trusted_pose_ = {};
  pending_map_to_odom_target_ = {};
  stabilization_anchor_pose_ = {};
  phase_ = RelocalizationPhase::kTracking;
  failed_attempts_ = 0;
  stabilization_observations_ = 0;
  recovery_action_ticks_ = 0;
  has_last_trusted_pose_ = false;
  has_pending_map_to_odom_target_ = false;
  has_stabilization_anchor_pose_ = false;
}

void RelocalizationManager::SetLastTrustedPose(const data::Pose3f& pose) {
  if (!pose.is_valid) {
    return;
  }
  last_trusted_pose_ = pose;
  has_last_trusted_pose_ = true;
}

void RelocalizationManager::UpdateLockState(common::TimePoint stamp, bool lock_lost) {
  if (!lock_lost) {
    if (phase_ == RelocalizationPhase::kLostLock || phase_ == RelocalizationPhase::kSearching) {
      phase_ = RelocalizationPhase::kTracking;
      lost_lock_started_at_ = {};
      failed_attempts_ = 0;
      recovery_action_ticks_ = 0;
      has_stabilization_anchor_pose_ = false;
    }
    return;
  }

  if (phase_ == RelocalizationPhase::kTracking) {
    phase_ = RelocalizationPhase::kLostLock;
    lost_lock_started_at_ = stamp;
    last_attempt_stamp_ = {};
    failed_attempts_ = 0;
    recovery_action_ticks_ = 0;
    ClearPendingMapToOdomTarget();
    has_stabilization_anchor_pose_ = false;
    ResetStabilizationWindow();
  }
}

void RelocalizationManager::ResetStabilizationWindow() {
  stabilization_started_at_ = {};
  stabilization_observations_ = 0;
}

void RelocalizationManager::SetPendingMapToOdomTarget(const data::Pose3f& map_to_odom) {
  pending_map_to_odom_target_ = map_to_odom;
  pending_map_to_odom_target_.reference_frame = tf::kMapFrame;
  pending_map_to_odom_target_.child_frame = tf::kOdomFrame;
  pending_map_to_odom_target_.is_valid = map_to_odom.is_valid;
  has_pending_map_to_odom_target_ = map_to_odom.is_valid;
}

void RelocalizationManager::ClearPendingMapToOdomTarget() {
  pending_map_to_odom_target_ = {};
  has_pending_map_to_odom_target_ = false;
}

data::Pose3f RelocalizationManager::BlendMapToOdomTarget(
    const data::Pose3f& current_map_to_odom) {
  if (!has_pending_map_to_odom_target_ || !pending_map_to_odom_target_.is_valid) {
    return current_map_to_odom;
  }
  if (!current_map_to_odom.is_valid) {
    has_pending_map_to_odom_target_ = false;
    return pending_map_to_odom_target_;
  }

  data::Pose3f blended = current_map_to_odom;
  const float dx = pending_map_to_odom_target_.position.x - current_map_to_odom.position.x;
  const float dy = pending_map_to_odom_target_.position.y - current_map_to_odom.position.y;
  const float translation = std::sqrt(dx * dx + dy * dy);
  const float translation_step = static_cast<float>(
      std::max(0.0, config_.relocalization_map_to_odom_apply_translation_step_m));
  if (translation > 1.0e-6F && translation_step > 0.0F) {
    const float scale = std::min(1.0F, translation_step / translation);
    blended.position.x += dx * scale;
    blended.position.y += dy * scale;
  } else {
    blended.position.x = pending_map_to_odom_target_.position.x;
    blended.position.y = pending_map_to_odom_target_.position.y;
  }

  const float yaw_delta =
      NormalizeAngle(pending_map_to_odom_target_.rpy.z - current_map_to_odom.rpy.z);
  const float yaw_step =
      static_cast<float>(std::max(0.0, config_.relocalization_map_to_odom_apply_yaw_step_rad));
  if (std::fabs(yaw_delta) > 1.0e-6F && yaw_step > 0.0F) {
    blended.rpy.z = NormalizeAngle(current_map_to_odom.rpy.z + ClampAbs(yaw_delta, yaw_step));
  } else {
    blended.rpy.z = pending_map_to_odom_target_.rpy.z;
  }
  blended.stamp = current_map_to_odom.stamp;
  blended.reference_frame = tf::kMapFrame;
  blended.child_frame = tf::kOdomFrame;
  blended.is_valid = true;

  const float remaining_dx = pending_map_to_odom_target_.position.x - blended.position.x;
  const float remaining_dy = pending_map_to_odom_target_.position.y - blended.position.y;
  const float remaining_translation = std::sqrt(remaining_dx * remaining_dx + remaining_dy * remaining_dy);
  const float remaining_yaw =
      std::fabs(NormalizeAngle(pending_map_to_odom_target_.rpy.z - blended.rpy.z));
  if (remaining_translation <= 1.0e-3F && remaining_yaw <= 1.0e-3F &&
      phase_ == RelocalizationPhase::kTracking) {
    has_pending_map_to_odom_target_ = false;
  }
  return blended;
}

RelocalizationStatus RelocalizationManager::CurrentStatus(common::TimePoint stamp) const {
  RelocalizationStatus status;
  FillStatus(stamp, &status);
  return status;
}

void RelocalizationManager::FillStatus(common::TimePoint stamp,
                                       RelocalizationStatus* status) const {
  if (status == nullptr) {
    return;
  }
  status->active = phase_ != RelocalizationPhase::kTracking;
  status->map_to_odom_blending_active = has_pending_map_to_odom_target_;
  status->phase = phase_;
  status->failed_attempts = failed_attempts_;
  status->stabilization_observations = stabilization_observations_;
  status->recovery_action = CurrentRecoveryAction();
  status->recovery_cmd = BuildRecoveryCommand(stamp);
  if (lost_lock_started_at_ != common::TimePoint{} && stamp != common::TimePoint{} &&
      stamp >= lost_lock_started_at_) {
    status->lost_lock_elapsed_ns = common::ToNanoseconds(stamp - lost_lock_started_at_);
  }
  if (stabilization_started_at_ != common::TimePoint{} && stamp != common::TimePoint{} &&
      stamp >= stabilization_started_at_) {
    status->stabilization_elapsed_ns = common::ToNanoseconds(stamp - stabilization_started_at_);
  }
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

StaticMap RelocalizationManager::BuildValidationSubmap(const StaticMap& map,
                                                       const data::Pose3f& center_pose) const {
  StaticMap submap = map;
  submap.global_points.clear();
  const float base_radius =
      static_cast<float>(std::max(0.5, config_.relocalization_submap_radius_m));
  const float step_m =
      static_cast<float>(std::max(0.2, config_.relocalization_linear_search_step_m));
  const float radius_m = std::max(base_radius * 1.5F, base_radius + step_m);
  const float radius_sq = radius_m * radius_m;
  for (const auto& point : map.global_points) {
    const float dx = point.x - center_pose.position.x;
    const float dy = point.y - center_pose.position.y;
    if (dx * dx + dy * dy <= radius_sq) {
      submap.global_points.push_back(point);
    }
  }

  const std::size_t base_max_points =
      static_cast<std::size_t>(std::max(1, config_.relocalization_submap_max_points));
  const std::size_t max_points = base_max_points * 2U;
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

std::vector<data::Pose3f> RelocalizationManager::BuildFixedPoseSet(
    const data::Pose3f& seed) const {
  std::vector<data::Pose3f> candidates;
  if (!seed.is_valid) {
    return candidates;
  }

  const float step_m =
      static_cast<float>(std::max(0.2, config_.relocalization_linear_search_step_m));
  const float yaw_step =
      static_cast<float>(std::max(0.05, config_.relocalization_yaw_search_step_rad));
  const std::array<std::tuple<float, float, float>, 8> fixed_offsets = {{
      {0.0F, 0.0F, 0.0F},
      {step_m, 0.0F, 0.0F},
      {-step_m, 0.0F, 0.0F},
      {0.0F, step_m, 0.0F},
      {0.0F, -step_m, 0.0F},
      {0.0F, 0.0F, yaw_step},
      {0.0F, 0.0F, -yaw_step},
      {0.0F, 0.0F, 3.14159265358979323846F},
  }};

  candidates.reserve(fixed_offsets.size());
  for (const auto& [dx, dy, dyaw] : fixed_offsets) {
    candidates.push_back(OffsetPose(seed, dx, dy, dyaw));
  }
  return candidates;
}

std::vector<data::Pose3f> RelocalizationManager::BuildCandidates(
    const data::Pose3f& predicted_guess, const data::Pose3f& last_trusted_guess,
    const data::Pose3f& fallback_guess) const {
  const float step_m =
      static_cast<float>(std::max(0.2, config_.relocalization_linear_search_step_m));
  const float yaw_step =
      static_cast<float>(std::max(0.05, config_.relocalization_yaw_search_step_rad));
  const std::array<std::tuple<float, float, float>, 11> local_offsets = {{
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
  candidates.reserve(local_offsets.size() * 4U);
  const auto append_seed = [&](const data::Pose3f& seed, bool include_fixed_set) {
    if (!seed.is_valid) {
      return;
    }
    for (const auto& [dx, dy, dyaw] : local_offsets) {
      candidates.push_back(OffsetPose(seed, dx, dy, dyaw));
    }
    if (include_fixed_set) {
      const auto fixed = BuildFixedPoseSet(seed);
      candidates.insert(candidates.end(), fixed.begin(), fixed.end());
    }
  };

  const auto action = CurrentRecoveryAction();
  append_seed(predicted_guess, false);
  append_seed(last_trusted_guess, action == RelocalizationRecoveryAction::kFixedPoseSweep);
  append_seed(fallback_guess, action == RelocalizationRecoveryAction::kFixedPoseSweep);

  if (action == RelocalizationRecoveryAction::kBackoffSpin ||
      action == RelocalizationRecoveryAction::kFixedPoseSweep) {
    const float backoff_step = std::max(0.1F, step_m * 0.5F);
    if (predicted_guess.is_valid) {
      candidates.push_back(OffsetPose(predicted_guess, -backoff_step, 0.0F, 0.0F));
    }
    if (last_trusted_guess.is_valid) {
      candidates.push_back(OffsetPose(last_trusted_guess, -backoff_step, 0.0F, 0.0F));
    }
  }

  return candidates;
}

RelocalizationRecoveryAction RelocalizationManager::CurrentRecoveryAction() const {
  if (phase_ == RelocalizationPhase::kTracking ||
      phase_ == RelocalizationPhase::kStabilizing) {
    return RelocalizationRecoveryAction::kNone;
  }
  if (failed_attempts_ >= 4) {
    return RelocalizationRecoveryAction::kFixedPoseSweep;
  }
  if (failed_attempts_ >= 2) {
    return RelocalizationRecoveryAction::kBackoffSpin;
  }
  return RelocalizationRecoveryAction::kSlowSpin;
}

data::ChassisCmd RelocalizationManager::BuildRecoveryCommand(common::TimePoint stamp) const {
  data::ChassisCmd cmd;
  cmd.stamp = stamp;
  cmd.brake = true;

  switch (CurrentRecoveryAction()) {
    case RelocalizationRecoveryAction::kSlowSpin:
      cmd.wz_radps = static_cast<float>(config_.relocalization_recovery_spin_wz_radps);
      cmd.brake = false;
      break;
    case RelocalizationRecoveryAction::kBackoffSpin:
      if (recovery_action_ticks_ < std::max(1, config_.relocalization_recovery_backoff_ticks)) {
        cmd.vx_mps = -static_cast<float>(std::max(0.0, config_.relocalization_recovery_backoff_vx_mps));
      } else {
        cmd.wz_radps = static_cast<float>(config_.relocalization_recovery_spin_wz_radps);
      }
      cmd.brake = false;
      break;
    case RelocalizationRecoveryAction::kFixedPoseSweep:
      cmd.wz_radps = static_cast<float>(config_.relocalization_recovery_spin_wz_radps * 0.7);
      cmd.brake = false;
      break;
    case RelocalizationRecoveryAction::kNone:
    default:
      break;
  }
  return cmd;
}

bool RelocalizationManager::IsStableObservationPose(const data::Pose3f& observed_pose) const {
  if (!observed_pose.is_valid) {
    return false;
  }
  if (!has_stabilization_anchor_pose_ || !stabilization_anchor_pose_.is_valid) {
    return true;
  }

  const float translation = PoseTranslationDistance(observed_pose, stabilization_anchor_pose_);
  const float yaw_gap =
      std::fabs(NormalizeAngle(observed_pose.rpy.z - stabilization_anchor_pose_.rpy.z));
  const float translation_limit = static_cast<float>(
      std::max(config_.max_position_jump_m, config_.relocalization_linear_search_step_m * 0.75));
  const float yaw_limit = static_cast<float>(
      std::max(config_.max_yaw_jump_rad, config_.relocalization_yaw_search_step_rad));
  return translation <= translation_limit && yaw_gap <= yaw_limit;
}

bool RelocalizationManager::IsAmbiguous(const CandidateMatch& best,
                                        const CandidateMatch& second_best) const {
  if (!best.valid || !second_best.valid || !best.match.converged || !second_best.match.converged) {
    return false;
  }
  const float score_gap = best.match.score - second_best.match.score;
  const float translation =
      PoseTranslationDistance(best.match.matched_pose, second_best.match.matched_pose);
  const float yaw_gap = std::fabs(
      NormalizeAngle(best.match.matched_pose.rpy.z - second_best.match.matched_pose.rpy.z));
  const bool clearly_separated =
      translation >= static_cast<float>(config_.relocalization_ambiguity_translation_m) ||
      yaw_gap >= static_cast<float>(config_.relocalization_ambiguity_yaw_rad);
  return score_gap <= static_cast<float>(config_.relocalization_ambiguity_score_gap) &&
         clearly_separated;
}

float RelocalizationManager::PoseConsistencyCost(const data::Pose3f& pose,
                                                 const data::Pose3f& predicted_guess,
                                                 const data::Pose3f& last_trusted_guess,
                                                 const data::Pose3f& fallback_guess) const {
  float best_cost = std::numeric_limits<float>::max();
  const auto consider_seed = [&](const data::Pose3f& seed) {
    if (!seed.is_valid) {
      return;
    }
    const float translation = PoseTranslationDistance(pose, seed);
    const float yaw_gap = std::fabs(NormalizeAngle(pose.rpy.z - seed.rpy.z));
    best_cost = std::min(best_cost, translation + 0.5F * yaw_gap);
  };

  consider_seed(predicted_guess);
  consider_seed(last_trusted_guess);
  consider_seed(fallback_guess);
  return best_cost;
}

bool RelocalizationManager::ResolveAmbiguity(const StaticMap& map, const data::LidarFrame& scan,
                                             const data::Pose3f& predicted_guess,
                                             const data::Pose3f& last_trusted_guess,
                                             const data::Pose3f& fallback_guess,
                                             CandidateMatch* best,
                                             CandidateMatch* second_best,
                                             RelocalizationStatus* status) const {
  if (best == nullptr || second_best == nullptr || status == nullptr) {
    return false;
  }

  status->secondary_check_performed = true;

  auto refine_candidate = [&](CandidateMatch* candidate) {
    if (candidate == nullptr || !candidate->valid || !candidate->match.matched_pose.is_valid) {
      return false;
    }
    const auto validation_submap = BuildValidationSubmap(map, candidate->match.matched_pose);
    if (validation_submap.global_points.empty()) {
      return false;
    }
    ScanMatchResult refined_match;
    const auto match_status = coarse_icp_matcher_.Match(validation_submap, scan,
                                                        candidate->match.matched_pose,
                                                        &refined_match);
    if (!match_status.ok() || !refined_match.converged) {
      return false;
    }
    candidate->match = refined_match;
    return true;
  };

  CandidateMatch refined_best = *best;
  CandidateMatch refined_second = *second_best;
  if (!refine_candidate(&refined_best) || !refine_candidate(&refined_second)) {
    return false;
  }

  if (refined_second.match.score > refined_best.match.score) {
    std::swap(refined_best, refined_second);
  }

  const float refined_score_gap = refined_best.match.score - refined_second.match.score;
  const float best_consistency = PoseConsistencyCost(refined_best.match.matched_pose,
                                                     predicted_guess, last_trusted_guess,
                                                     fallback_guess);
  const float second_consistency = PoseConsistencyCost(refined_second.match.matched_pose,
                                                       predicted_guess, last_trusted_guess,
                                                       fallback_guess);
  const float consistency_margin =
      std::max(0.25F,
               static_cast<float>(std::max(0.2, config_.relocalization_linear_search_step_m)) *
                   0.5F);
  const bool score_resolved =
      refined_score_gap > static_cast<float>(config_.relocalization_ambiguity_score_gap);
  const bool consistency_resolved =
      best_consistency + consistency_margin < second_consistency;
  if (!score_resolved && !consistency_resolved) {
    return false;
  }

  *best = refined_best;
  *second_best = refined_second;
  status->secondary_check_passed = true;
  return true;
}

void RelocalizationManager::RecordStabilizationObservation(common::TimePoint stamp,
                                                           const data::Pose3f& observed_pose,
                                                           bool pose_trusted,
                                                           RelocalizationStatus* status) {
  if (phase_ != RelocalizationPhase::kStabilizing) {
    if (status != nullptr) {
      FillStatus(stamp, status);
    }
    return;
  }

  if (!pose_trusted || !IsStableObservationPose(observed_pose)) {
    phase_ = RelocalizationPhase::kSearching;
    last_attempt_stamp_ = {};
    ClearPendingMapToOdomTarget();
    has_stabilization_anchor_pose_ = false;
    ResetStabilizationWindow();
    ++failed_attempts_;
    ++recovery_action_ticks_;
    if (status != nullptr) {
      status->stabilization_failed = true;
      FillStatus(stamp, status);
    }
    return;
  }

  if (stabilization_observations_ == 0) {
    stabilization_started_at_ = stamp;
  }
  stabilization_anchor_pose_ = observed_pose;
  has_stabilization_anchor_pose_ = observed_pose.is_valid;
  ++stabilization_observations_;
  if (status != nullptr) {
    FillStatus(stamp, status);
  }

  const int required_frames = std::max(1, config_.relocalization_stabilization_frames);
  bool time_ready = config_.relocalization_stabilization_time_ms <= 0;
  if (!time_ready && stabilization_started_at_ != common::TimePoint{} && stamp >= stabilization_started_at_) {
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(stamp - stabilization_started_at_)
            .count();
    time_ready = elapsed_ms >= config_.relocalization_stabilization_time_ms;
  }

  if (stabilization_observations_ >= required_frames && time_ready) {
    phase_ = RelocalizationPhase::kTracking;
    lost_lock_started_at_ = {};
    failed_attempts_ = 0;
    recovery_action_ticks_ = 0;
    has_stabilization_anchor_pose_ = false;
    ResetStabilizationWindow();
    if (status != nullptr) {
      FillStatus(stamp, status);
    }
  }
}

common::Status RelocalizationManager::Attempt(const StaticMap& map, const data::LidarFrame& scan,
                                              const data::Pose3f& predicted_guess,
                                              const data::Pose3f& last_trusted_guess,
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
  FillStatus(stamp, status);
  if (phase_ == RelocalizationPhase::kTracking) {
    return common::Status::Ok();
  }
  if (!map.global_map_loaded || map.global_points.empty() || scan.points.empty()) {
    status->matcher_used = "none";
    return common::Status::NotReady("relocalization needs map and scan points");
  }

  if (phase_ == RelocalizationPhase::kLostLock) {
    phase_ = RelocalizationPhase::kSearching;
    FillStatus(stamp, status);
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
  const data::Pose3f trusted_guess =
      last_trusted_guess.is_valid ? last_trusted_guess : last_trusted_pose_;
  const data::Pose3f fallback_seed =
      fallback_guess.is_valid ? fallback_guess : last_trusted_pose_;
  const auto candidates = BuildCandidates(predicted_guess, trusted_guess, fallback_seed);
  CandidateMatch best_candidate;
  CandidateMatch second_best;

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
    CandidateMatch candidate_result;
    candidate_result.match = candidate_match;
    candidate_result.initial_guess = candidate;
    candidate_result.matcher_used = matcher_used;
    candidate_result.fallback_used = fallback_used;
    candidate_result.valid = true;

    if (!best_candidate.valid || candidate_match.score > best_candidate.match.score) {
      second_best = best_candidate;
      best_candidate = candidate_result;
    } else if (!second_best.valid || candidate_match.score > second_best.match.score) {
      second_best = candidate_result;
    }
  }

  status->attempt_latency_ns = common::NowNs() - begin_ns;
  if (!best_candidate.valid) {
    ++failed_attempts_;
    ++recovery_action_ticks_;
    FillStatus(stamp, status);
    status->attempted = true;
    return common::Status::Ok();
  }

  status->best_initial_guess = best_candidate.initial_guess;
  status->matched_pose = best_candidate.match.matched_pose;
  status->best_score = best_candidate.match.score;
  status->fallback_matcher_used = best_candidate.fallback_used;
  status->matcher_used = best_candidate.matcher_used;

  if (IsAmbiguous(best_candidate, second_best)) {
    if (!ResolveAmbiguity(map, scan, predicted_guess, trusted_guess, fallback_seed,
                          &best_candidate, &second_best, status)) {
      status->ambiguity_rejected = true;
      ++failed_attempts_;
      ++recovery_action_ticks_;
      FillStatus(stamp, status);
      status->attempted = true;
      status->best_initial_guess = best_candidate.initial_guess;
      status->matched_pose = best_candidate.match.matched_pose;
      status->best_score = best_candidate.match.score;
      status->matcher_used = best_candidate.matcher_used;
      return common::Status::Ok();
    }
    status->best_initial_guess = best_candidate.initial_guess;
    status->matched_pose = best_candidate.match.matched_pose;
    status->best_score = best_candidate.match.score;
    status->fallback_matcher_used = best_candidate.fallback_used;
    status->matcher_used = best_candidate.matcher_used;
  }

  if (!best_candidate.match.converged ||
      best_candidate.match.score <
          static_cast<float>(config_.relocalization_min_match_score)) {
    ++failed_attempts_;
    ++recovery_action_ticks_;
    FillStatus(stamp, status);
    status->attempted = true;
    return common::Status::Ok();
  }

  *best_match = best_candidate.match;
  phase_ = RelocalizationPhase::kStabilizing;
  failed_attempts_ = 0;
  recovery_action_ticks_ = 0;
  ResetStabilizationWindow();
  stabilization_anchor_pose_ = best_candidate.match.matched_pose;
  has_stabilization_anchor_pose_ = best_candidate.match.matched_pose.is_valid;
  status->succeeded = true;
  FillStatus(stamp, status);
  status->attempted = true;
  status->succeeded = true;
  status->best_initial_guess = best_candidate.initial_guess;
  status->matched_pose = best_candidate.match.matched_pose;
  status->best_score = best_candidate.match.score;
  status->fallback_matcher_used = best_candidate.fallback_used;
  status->matcher_used = best_candidate.matcher_used;
  return common::Status::Ok();
}

}  // namespace rm_nav::localization
