#include "rm_nav/mapping/mapping_engine.hpp"

#include <cmath>
#include <filesystem>
#include <limits>
#include <utility>

#include "rm_nav/common/time.hpp"
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

float ClampAbs(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

float PoseTranslationError(const data::Pose3f& left, const data::Pose3f& right) {
  const float dx = left.position.x - right.position.x;
  const float dy = left.position.y - right.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

float PoseYawError(const data::Pose3f& left, const data::Pose3f& right) {
  return NormalizeAngle(left.rpy.z - right.rpy.z);
}

data::Pose3f ComposePose2D(const data::Pose3f& parent, const data::Pose3f& relative) {
  data::Pose3f composed = parent;
  const float cos_yaw = std::cos(parent.rpy.z);
  const float sin_yaw = std::sin(parent.rpy.z);
  composed.position.x =
      parent.position.x + cos_yaw * relative.position.x - sin_yaw * relative.position.y;
  composed.position.y =
      parent.position.y + sin_yaw * relative.position.x + cos_yaw * relative.position.y;
  composed.position.z = parent.position.z + relative.position.z;
  composed.rpy.x = parent.rpy.x + relative.rpy.x;
  composed.rpy.y = parent.rpy.y + relative.rpy.y;
  composed.rpy.z = NormalizeAngle(parent.rpy.z + relative.rpy.z);
  composed.is_valid = parent.is_valid && relative.is_valid;
  return composed;
}

data::Pose3f MakeZeroCorrection() {
  data::Pose3f correction;
  correction.reference_frame = tf::kMapFrame;
  correction.child_frame = tf::kBaseLinkFrame;
  correction.is_valid = true;
  return correction;
}

data::Pose3f RelativePose2D(const data::Pose3f& reference, const data::Pose3f& current) {
  data::Pose3f relative = MakeZeroCorrection();
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
  relative.rpy.z = NormalizeAngle(current.rpy.z - reference.rpy.z);
  return relative;
}

data::LidarFrame DownsampleLidarFrame(const data::LidarFrame& frame, int max_points) {
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

std::vector<data::PointXYZI> BuildLocalSubmap(const std::vector<data::PointXYZI>& global_points,
                                              const data::Pose3f& center_pose,
                                              float radius_m, int max_points) {
  std::vector<data::PointXYZI> local_points;
  const float radius_sq = radius_m * radius_m;
  for (const auto& point : global_points) {
    const float dx = point.x - center_pose.position.x;
    const float dy = point.y - center_pose.position.y;
    if (dx * dx + dy * dy <= radius_sq) {
      local_points.push_back(point);
    }
  }

  const std::size_t point_limit = static_cast<std::size_t>(std::max(1, max_points));
  if (local_points.size() <= point_limit) {
    return local_points;
  }

  std::vector<data::PointXYZI> downsampled;
  downsampled.reserve(point_limit);
  for (std::size_t index = 0; index < point_limit; ++index) {
    const std::size_t source_index = (index * local_points.size()) / point_limit;
    downsampled.push_back(local_points[source_index]);
  }
  return downsampled;
}

data::Pose3f PredictPoseFromExternalDelta(const data::Pose3f& previous_external_pose,
                                          const data::Pose3f& previous_mapping_pose,
                                          const data::Pose3f& current_external_pose,
                                          const data::PreintegratedImuBlock& preint) {
  auto relative = RelativePose2D(previous_external_pose, current_external_pose);
  if (preint.is_valid) {
    relative.rpy.z = NormalizeAngle(preint.delta_rpy.z);
  }
  return ComposePose2D(previous_mapping_pose, relative);
}

MapStorageLayout BuildStorageLayout(const config::MappingConfig& config,
                                    const std::string& active_dir_override) {
  const std::filesystem::path active_dir =
      active_dir_override.empty() ? std::filesystem::path(config.active_dir)
                                  : std::filesystem::path(active_dir_override);
  const auto parent_dir = active_dir.parent_path();

  MapStorageLayout layout;
  layout.active_dir = active_dir;
  layout.staging_dir = config.staging_dir.empty() ? (parent_dir / "staging")
                                                  : std::filesystem::path(config.staging_dir);
  layout.failed_dir = config.failed_dir.empty() ? (parent_dir / "failed")
                                                : std::filesystem::path(config.failed_dir);
  return layout;
}

}  // namespace

common::Status MappingEngine::Initialize(const config::MappingConfig& config) {
  config_ = config;
  latest_result_ = {};
  keyframes_.clear();
  trajectory_samples_.clear();
  latest_loop_candidate_ = {};
  latest_loop_match_ = {};
  latest_loop_correction_ = {};
  consecutive_loop_correction_failures_ = 0;
  applied_pose_correction_ = MakeZeroCorrection();
  target_pose_correction_ = MakeZeroCorrection();
  previous_external_pose_ = {};
  previous_mapping_pose_ = {};
  previous_scan_ = {};
  has_previous_frontend_state_ = false;
  auto status = builder_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  localization::ScanMatchConfig frontend_config;
  frontend_config.max_iterations = config.frontend_match_max_iterations;
  frontend_config.correspondence_distance_m =
      static_cast<float>(config.frontend_correspondence_distance_m);
  frontend_config.min_match_score = static_cast<float>(config.frontend_min_match_score);
  status = frontend_icp_matcher_.Configure(frontend_config);
  if (!status.ok()) {
    return status;
  }
  status = lio_frontend_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  status = projector_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  status = loop_closure_matcher_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  initialized_ = true;
  return common::Status::Ok();
}

common::Status MappingEngine::Update(const data::SyncedFrame& frame,
                                     const data::Pose3f& map_to_base) {
  const std::vector<data::DynamicObstacle> no_dynamic_obstacles;
  return Update(frame, map_to_base, no_dynamic_obstacles);
}

common::Status MappingEngine::Update(
    const data::SyncedFrame& frame, const data::Pose3f& map_to_base,
    const std::vector<data::DynamicObstacle>& known_dynamic_obstacles) {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  data::SyncedFrame frontend_frame;
  auto status = lio_frontend_.PrepareFrame(frame, &frontend_frame);
  if (!status.ok()) {
    return status;
  }
  bool pose_is_map_aligned = false;
  data::Pose3f frontend_pose;
  status = ResolveMappingPose(frontend_frame, map_to_base, &frontend_pose, &pose_is_map_aligned);
  if (!status.ok()) {
    return status;
  }
  AdvanceCorrectionTowardTarget();
  const data::Pose3f corrected_map_to_base =
      pose_is_map_aligned ? frontend_pose : ApplyCorrection(frontend_pose);
  const auto begin_ns = common::NowNs();
  status = builder_.Update(frontend_frame, corrected_map_to_base, known_dynamic_obstacles);
  if (!status.ok()) {
    return status;
  }
  latest_result_.map_to_base = corrected_map_to_base;
  latest_result_.accumulated_points = builder_.GlobalPoints().size();
  latest_result_.processed_frames = builder_.frame_count();
  latest_result_.processing_latency_ns = common::NowNs() - begin_ns;
  status = loop_candidate_detector_.FindCandidate(config_, corrected_map_to_base, frontend_frame.stamp,
                                                  frontend_frame.lidar.frame_index, keyframes_,
                                                  &latest_loop_candidate_);
  if (!status.ok()) {
    return status;
  }
  latest_loop_match_ = {};
  const bool loop_correction_disabled =
      consecutive_loop_correction_failures_ >=
      std::max(1, config_.loop_correction_max_consecutive_failures);
  if (latest_loop_candidate_.found && loop_correction_disabled) {
    latest_loop_match_.candidate_found = true;
    latest_loop_match_.keyframe_index = latest_loop_candidate_.keyframe_index;
    latest_loop_match_.frame_index = latest_loop_candidate_.frame_index;
    latest_loop_match_.status_code = common::StatusCode::kUnavailable;
    latest_loop_match_.status_message = "loop correction auto-disabled after failures";
  } else if (latest_loop_candidate_.found &&
      latest_loop_candidate_.keyframe_index < keyframes_.size()) {
    const auto& candidate_keyframe = keyframes_[latest_loop_candidate_.keyframe_index];
    status = loop_closure_matcher_.Match(frontend_frame, corrected_map_to_base, candidate_keyframe,
                                         latest_loop_candidate_, &latest_loop_match_);
    if (!status.ok()) {
      return status;
    }
  }
  status = loop_correction_gate_.Evaluate(config_, latest_loop_candidate_, latest_loop_match_,
                                          consecutive_loop_correction_failures_,
                                          &latest_loop_correction_);
  if (!status.ok()) {
    return status;
  }
  consecutive_loop_correction_failures_ =
      latest_loop_correction_.consecutive_failures_after;
  if (latest_loop_correction_.accepted &&
      latest_loop_candidate_.keyframe_index < keyframes_.size()) {
    UpdateCorrectionTarget(frontend_pose, keyframes_[latest_loop_candidate_.keyframe_index],
                           latest_loop_correction_);
  }
  RecordTrajectorySample(frontend_frame, map_to_base, latest_result_.predicted_pose,
                         corrected_map_to_base);
  if (ShouldCaptureKeyframe(frontend_frame, corrected_map_to_base)) {
    CaptureKeyframe(frontend_frame, corrected_map_to_base);
  }
  previous_external_pose_ = map_to_base;
  previous_mapping_pose_ = corrected_map_to_base;
  previous_scan_ = DownsampleLidarFrame(frontend_frame.lidar, config_.frontend_match_max_points);
  has_previous_frontend_state_ = true;
  return common::Status::Ok();
}

common::Status MappingEngine::BuildGridMap2D(data::GridMap2D* grid_map) const {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  return projector_.Project(builder_.GlobalPoints(), keyframes_, grid_map);
}

common::Status MappingEngine::SaveMap(const std::string& active_dir_override,
                                      localization::StaticMap* exported_map,
                                      MapSaveFailureKind* failure_kind) const {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  data::GridMap2D grid_map;
  auto status = BuildGridMap2D(&grid_map);
  if (!status.ok()) {
    return status;
  }

  const auto global_points = builder_.GlobalPoints();
  const std::string resolved_active_dir =
      active_dir_override.empty() ? config_.active_dir : active_dir_override;
  const auto storage_layout = BuildStorageLayout(config_, resolved_active_dir);
  mapping::MapArtifactPaths artifacts;
  status = save_manager_.SaveAndActivate(storage_layout, config_, global_points, grid_map,
                                         trajectory_samples_,
                                         &artifacts, nullptr, failure_kind);
  if (!status.ok()) {
    return status;
  }

  if (exported_map != nullptr) {
    exported_map->occupancy = grid_map;
    exported_map->global_points = global_points;
    exported_map->occupancy_path = artifacts.occupancy_bin_path;
    exported_map->global_map_path = artifacts.global_map_pcd_path;
    exported_map->occupancy_loaded = true;
    exported_map->global_map_loaded = true;
  }
  return common::Status::Ok();
}

bool MappingEngine::ShouldCaptureKeyframe(const data::SyncedFrame& frame,
                                          const data::Pose3f& map_to_base) const {
  if (!map_to_base.is_valid || frame.lidar.points.empty()) {
    return false;
  }
  if (keyframes_.empty()) {
    return true;
  }

  const auto& last_keyframe = keyframes_.back();
  const float dx = map_to_base.position.x - last_keyframe.map_to_base.position.x;
  const float dy = map_to_base.position.y - last_keyframe.map_to_base.position.y;
  const float translation = std::sqrt(dx * dx + dy * dy);
  const float yaw_delta =
      std::fabs(NormalizeAngle(map_to_base.rpy.z - last_keyframe.map_to_base.rpy.z));

  return translation >= static_cast<float>(config_.keyframe_translation_threshold_m) ||
         yaw_delta >= static_cast<float>(config_.keyframe_yaw_threshold_rad);
}

void MappingEngine::CaptureKeyframe(const data::SyncedFrame& frame,
                                    const data::Pose3f& map_to_base) {
  MappingKeyframe keyframe;
  keyframe.stamp = frame.stamp;
  keyframe.frame_index = frame.lidar.frame_index;
  keyframe.map_to_base = map_to_base;

  const std::size_t point_limit =
      static_cast<std::size_t>(std::max(1, config_.keyframe_max_points));
  keyframe.local_points.reserve(std::min(point_limit, frame.lidar.points.size()));
  if (frame.lidar.points.size() <= point_limit) {
    keyframe.local_points = frame.lidar.points.view();
  } else {
    for (std::size_t index = 0; index < point_limit; ++index) {
      const std::size_t source_index = (index * frame.lidar.points.size()) / point_limit;
      keyframe.local_points.push_back(frame.lidar.points[source_index]);
    }
  }

  keyframes_.push_back(std::move(keyframe));
  const std::size_t max_keyframes =
      static_cast<std::size_t>(std::max(1, config_.max_keyframes));
  if (keyframes_.size() > max_keyframes) {
    keyframes_.erase(keyframes_.begin(),
                     keyframes_.begin() +
                         static_cast<std::ptrdiff_t>(keyframes_.size() - max_keyframes));
  }
}

data::Pose3f MappingEngine::ApplyCorrection(const data::Pose3f& raw_pose) const {
  data::Pose3f corrected = raw_pose;
  corrected.position.x += applied_pose_correction_.position.x;
  corrected.position.y += applied_pose_correction_.position.y;
  corrected.position.z += applied_pose_correction_.position.z;
  corrected.rpy.x += applied_pose_correction_.rpy.x;
  corrected.rpy.y += applied_pose_correction_.rpy.y;
  corrected.rpy.z = NormalizeAngle(corrected.rpy.z + applied_pose_correction_.rpy.z);
  corrected.is_valid = raw_pose.is_valid && applied_pose_correction_.is_valid;
  return corrected;
}

void MappingEngine::AdvanceCorrectionTowardTarget() {
  const float dx = target_pose_correction_.position.x - applied_pose_correction_.position.x;
  const float dy = target_pose_correction_.position.y - applied_pose_correction_.position.y;
  const float distance = std::sqrt(dx * dx + dy * dy);
  const float translation_step_limit =
      static_cast<float>(std::max(0.0, config_.loop_correction_apply_translation_step_m));
  if (distance > 1.0e-6F && translation_step_limit > 0.0F) {
    const float scale = std::min(1.0F, translation_step_limit / distance);
    applied_pose_correction_.position.x += dx * scale;
    applied_pose_correction_.position.y += dy * scale;
  } else {
    applied_pose_correction_.position.x = target_pose_correction_.position.x;
    applied_pose_correction_.position.y = target_pose_correction_.position.y;
  }

  const float yaw_delta =
      NormalizeAngle(target_pose_correction_.rpy.z - applied_pose_correction_.rpy.z);
  const float yaw_step_limit =
      static_cast<float>(std::max(0.0, config_.loop_correction_apply_yaw_step_rad));
  if (std::fabs(yaw_delta) > 1.0e-6F && yaw_step_limit > 0.0F) {
    applied_pose_correction_.rpy.z =
        NormalizeAngle(applied_pose_correction_.rpy.z +
                       ClampAbs(yaw_delta, yaw_step_limit));
  } else {
    applied_pose_correction_.rpy.z = target_pose_correction_.rpy.z;
  }
}

void MappingEngine::UpdateCorrectionTarget(const data::Pose3f& raw_pose,
                                           const MappingKeyframe& keyframe,
                                           const LoopCorrectionDecision& decision) {
  if (!decision.accepted || !decision.accepted_pose_candidate_frame.is_valid) {
    return;
  }

  const data::Pose3f target_map_to_base =
      ComposePose2D(keyframe.map_to_base, decision.accepted_pose_candidate_frame);
  target_pose_correction_ = MakeZeroCorrection();
  target_pose_correction_.stamp = raw_pose.stamp;
  target_pose_correction_.position.x = target_map_to_base.position.x - raw_pose.position.x;
  target_pose_correction_.position.y = target_map_to_base.position.y - raw_pose.position.y;
  target_pose_correction_.position.z = target_map_to_base.position.z - raw_pose.position.z;
  target_pose_correction_.rpy.x = target_map_to_base.rpy.x - raw_pose.rpy.x;
  target_pose_correction_.rpy.y = target_map_to_base.rpy.y - raw_pose.rpy.y;
  target_pose_correction_.rpy.z =
      NormalizeAngle(target_map_to_base.rpy.z - raw_pose.rpy.z);
}

void MappingEngine::RecordTrajectorySample(const data::SyncedFrame& frame,
                                           const data::Pose3f& external_pose,
                                           const data::Pose3f& predicted_pose,
                                           const data::Pose3f& optimized_pose) {
  MappingTrajectorySample sample;
  sample.stamp = frame.stamp;
  sample.frame_index = frame.lidar.frame_index;
  sample.external_pose = external_pose;
  sample.predicted_pose = predicted_pose;
  sample.optimized_pose = optimized_pose;
  sample.loop_candidate_found = latest_loop_candidate_.found;
  sample.loop_match_converged = latest_loop_match_.converged;
  sample.loop_correction_accepted = latest_loop_correction_.accepted;

  if (latest_loop_candidate_.found && latest_loop_match_.converged &&
      latest_loop_candidate_.keyframe_index < keyframes_.size()) {
    const auto& keyframe = keyframes_[latest_loop_candidate_.keyframe_index];
    const data::Pose3f target_map_to_base =
        ComposePose2D(keyframe.map_to_base, latest_loop_match_.matched_pose_candidate_frame);
    sample.loop_consistency_evaluated = true;
    sample.raw_loop_translation_error_m =
        PoseTranslationError(external_pose, target_map_to_base);
    sample.raw_loop_yaw_error_rad = PoseYawError(external_pose, target_map_to_base);
    sample.optimized_loop_translation_error_m =
        PoseTranslationError(optimized_pose, target_map_to_base);
    sample.optimized_loop_yaw_error_rad = PoseYawError(optimized_pose, target_map_to_base);
  }

  trajectory_samples_.push_back(std::move(sample));
}

common::Status MappingEngine::ResolveMappingPose(const data::SyncedFrame& frame,
                                                 const data::Pose3f& external_pose,
                                                 data::Pose3f* mapping_pose,
                                                 bool* pose_is_map_aligned) {
  if (mapping_pose == nullptr || pose_is_map_aligned == nullptr) {
    return common::Status::InvalidArgument("mapping pose outputs must not be null");
  }

  latest_result_.pose_source = config_.pose_source;
  latest_result_.frontend_score = 0.0F;
  latest_result_.frontend_iterations = 0;
  latest_result_.frontend_converged = false;
  latest_result_.frontend_fallback = false;
  latest_result_.frontend_latency_ns = 0;
  latest_result_.frontend_reference_points = 0;
  latest_result_.external_pose = external_pose;
  latest_result_.predicted_pose = external_pose;

  *mapping_pose = external_pose;
  *pose_is_map_aligned = false;
  if (!external_pose.is_valid || frame.lidar.points.empty()) {
    latest_result_.frontend_fallback = true;
    return common::Status::Ok();
  }

  if (config_.pose_source == "odom") {
    latest_result_.frontend_fallback = true;
    return common::Status::Ok();
  }

  const auto begin_ns = common::NowNs();
  LioFrontendPrediction frontend_prediction;
  if (has_previous_frontend_state_) {
    auto status = lio_frontend_.PredictPose(previous_external_pose_, previous_mapping_pose_,
                                            external_pose, frame.preint, &frontend_prediction);
    if (!status.ok()) {
      return status;
    }
  } else {
    frontend_prediction.predicted_pose = external_pose;
  }
  const data::Pose3f predicted_pose = frontend_prediction.predicted_pose;
  latest_result_.predicted_pose = predicted_pose;

  localization::StaticMap local_map;
  data::Pose3f initial_guess = external_pose;

  const bool scan_to_scan_mode =
      config_.pose_source == "scan_to_scan_icp" ||
      config_.pose_source == "lio_lite_scan_to_scan";
  const bool scan_to_map_mode =
      config_.pose_source == "scan_to_map_icp" ||
      config_.pose_source == "lio_lite_scan_to_map";
  const bool eskf_only_mode = config_.pose_source == "eskf_lite";
  if (eskf_only_mode) {
    latest_result_.frontend_latency_ns = common::NowNs() - begin_ns;
    latest_result_.frontend_converged = frontend_prediction.used_imu_prediction;
    *mapping_pose = predicted_pose;
    *pose_is_map_aligned = true;
    return common::Status::Ok();
  }

  if (scan_to_scan_mode) {
    if (!has_previous_frontend_state_ || previous_scan_.points.empty()) {
      latest_result_.frontend_fallback = true;
      latest_result_.frontend_latency_ns = common::NowNs() - begin_ns;
      *mapping_pose = predicted_pose;
      *pose_is_map_aligned = true;
      return common::Status::Ok();
    }
    local_map.global_points = previous_scan_.points.view();
    local_map.global_map_loaded = true;
    latest_result_.frontend_reference_points = local_map.global_points.size();
    initial_guess = RelativePose2D(previous_external_pose_, external_pose);
    if (frontend_prediction.used_imu_prediction) {
      initial_guess = RelativePose2D(previous_mapping_pose_, predicted_pose);
    }
  } else if (scan_to_map_mode) {
    local_map.global_points = BuildLocalSubmap(
        builder_.GlobalPoints(), predicted_pose,
        static_cast<float>(std::max(0.1, config_.frontend_submap_radius_m)),
        config_.frontend_submap_max_points);
    local_map.global_map_loaded = true;
    latest_result_.frontend_reference_points = local_map.global_points.size();
    initial_guess = predicted_pose;
    if (local_map.global_points.size() <
        static_cast<std::size_t>(std::max(1, config_.frontend_min_map_points))) {
      latest_result_.frontend_fallback = true;
      latest_result_.frontend_latency_ns = common::NowNs() - begin_ns;
      *mapping_pose = predicted_pose;
      *pose_is_map_aligned = true;
      return common::Status::Ok();
    }
  } else {
    return common::Status::InvalidArgument("unsupported mapping pose_source");
  }

  const auto scan = DownsampleLidarFrame(frame.lidar, config_.frontend_match_max_points);
  localization::ScanMatchResult match;
  auto status = frontend_icp_matcher_.Match(local_map, scan, initial_guess, &match);
  latest_result_.frontend_latency_ns = common::NowNs() - begin_ns;
  if (!status.ok()) {
    latest_result_.frontend_fallback = true;
    *mapping_pose = predicted_pose;
    *pose_is_map_aligned = true;
    return common::Status::Ok();
  }

  latest_result_.frontend_score = match.score;
  latest_result_.frontend_iterations = match.iterations;
  latest_result_.frontend_converged = match.converged;

  if (!match.converged) {
    latest_result_.frontend_fallback = true;
    *mapping_pose = predicted_pose;
    *pose_is_map_aligned = true;
    return common::Status::Ok();
  }

  if (scan_to_scan_mode) {
    *mapping_pose = ComposePose2D(previous_mapping_pose_, match.matched_pose);
  } else {
    *mapping_pose = match.matched_pose;
  }
  *pose_is_map_aligned = true;
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
