#include "rm_nav/localization/localization_engine.hpp"

#include <cmath>
#include <sstream>

#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/utils/logger.hpp"

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

data::PointXYZI TransformPoint(const data::PointXYZI& point, const data::Pose3f& pose) {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  data::PointXYZI transformed = point;
  transformed.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  transformed.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  transformed.z = pose.position.z + point.z;
  return transformed;
}

const char* ToString(RelocalizationPhase phase) {
  switch (phase) {
    case RelocalizationPhase::kTracking:
      return "tracking";
    case RelocalizationPhase::kLostLock:
      return "lost_lock";
    case RelocalizationPhase::kSearching:
      return "searching";
    case RelocalizationPhase::kStabilizing:
      return "stabilizing";
    default:
      return "unknown";
  }
}

void IncrementRejectionCounter(const std::string& reason,
                               LocalizationRejectionCounters* counters) {
  if (counters == nullptr || reason == "none" || reason.empty()) {
    return;
  }
  ++counters->total;
  if (reason == "map_unloaded") {
    ++counters->map_unloaded;
  } else if (reason == "matcher_not_converged") {
    ++counters->matcher_not_converged;
  } else if (reason == "low_match_score") {
    ++counters->low_match_score;
  } else if (reason == "pose_jump_guard") {
    ++counters->pose_jump_guard;
  } else if (reason == "consecutive_failures") {
    ++counters->consecutive_failures;
  } else if (reason == "map_to_odom_guard") {
    ++counters->map_to_odom_guard;
  } else if (reason == "relocalization_stabilization_failed") {
    ++counters->relocalization_stabilization_failed;
  } else if (reason == "stabilization_window") {
    ++counters->stabilization_window;
  } else {
    ++counters->other;
  }
}

}  // namespace

common::Status LocalizationEngine::Initialize(
    const std::string& config_dir, const config::LocalizationConfig& localization_config,
    const config::SpawnConfig& spawn_config, tf::TfTreeLite* tf_tree) {
  tf_tree_ = tf_tree;
  config_ = localization_config;
  initial_pose_provider_ = InitialPoseProvider(spawn_config);
  quality_estimator_ = PoseQualityEstimator(localization_config);
  processed_frames_ = 0;
  dropped_frames_ = 0;
  consecutive_failures_ = 0;
  has_latest_odom_ = false;
  has_trusted_map_to_odom_ = false;
  last_good_stamp_ = {};
  consecutive_rejections_ = 0;
  rejection_counters_ = {};
  last_trusted_map_to_base_ = {};
  static_map_ = {};

  ScanMatchConfig matcher_config;
  matcher_config.max_iterations = localization_config.max_iterations;
  matcher_config.correspondence_distance_m =
      static_cast<float>(localization_config.correspondence_distance_m);
  matcher_config.min_match_score =
      static_cast<float>(localization_config.min_match_score);

  if (localization_config.matcher == "ndt") {
    matcher_ = &ndt_matcher_;
  } else {
    matcher_ = &icp_matcher_;
  }
  current_matcher_label_ = localization_config.matcher;
  last_logged_matcher_label_ = current_matcher_label_;
  auto status = matcher_->Configure(matcher_config);
  if (!status.ok()) {
    return status;
  }

  if (config_.enabled) {
    MapLoader loader;
    status = loader.Load(config_dir, localization_config, &static_map_);
    if (!status.ok()) {
      return status;
    }
  }
  status = relocalization_manager_.Configure(localization_config);
  if (!status.ok()) {
    return status;
  }

  const auto now = common::Now();
  const data::Pose3f initial_map_to_base = initial_pose_provider_.MakeInitialPose(now);
  data::Pose3f initial_odom_to_base =
      tf::MakeTransform(tf::kOdomFrame, tf::kBaseLinkFrame, 0.0F, 0.0F, 0.0F, now);
  data::Pose3f initial_map_to_odom{};
  status = map_odom_fuser_.Fuse(initial_odom_to_base, initial_map_to_base, &initial_map_to_odom);
  if (!status.ok()) {
    return status;
  }

  LocalizationResult initial_result;
  initial_result.map_to_odom = initial_map_to_odom;
  initial_result.map_to_base = initial_map_to_base;
  initial_result.odom_to_base = initial_odom_to_base;
  initial_result.status.map_loaded = static_map_.global_map_loaded;
  initial_result.status.pose_trusted = false;
  initial_result.matcher_name = current_matcher_label_;
  latest_result_.Publish(initial_result);
  map_to_odom_.Publish(initial_map_to_odom);
  odom_to_base_.Publish(initial_odom_to_base);
  latest_aligned_scan_.Publish({});
  last_trusted_map_to_odom_ = initial_map_to_odom;
  has_trusted_map_to_odom_ = true;

  if (tf_tree_ != nullptr) {
    tf_tree_->PushMapToOdom(initial_map_to_odom);
    tf_tree_->PushOdomToBase(initial_odom_to_base);
  }
  std::ostringstream stream;
  stream << "initialized map=" << static_map_.version_label
         << " matcher=" << current_matcher_label_
         << " map_loaded=" << (static_map_.global_map_loaded ? "true" : "false");
  utils::LogInfo("localization", stream.str());
  return common::Status::Ok();
}

common::Status LocalizationEngine::ConfigureMatcherForLoad(bool light_mode) {
  if (matcher_ == nullptr || matcher_light_mode_active_ == light_mode) {
    return common::Status::Ok();
  }
  ScanMatchConfig matcher_config;
  matcher_config.max_iterations =
      light_mode ? std::max(1, config_.reduced_max_iterations) : config_.max_iterations;
  matcher_config.correspondence_distance_m =
      static_cast<float>(config_.correspondence_distance_m);
  matcher_config.min_match_score = static_cast<float>(config_.min_match_score);
  auto status = matcher_->Configure(matcher_config);
  if (!status.ok()) {
    return status;
  }
  matcher_light_mode_active_ = light_mode;
  current_matcher_label_ = config_.matcher + std::string(light_mode ? "_light" : "");
  return common::Status::Ok();
}

common::Status LocalizationEngine::EnqueueFrame(sync::SyncedFrameHandle frame) {
  if (!input_queue_.try_push(std::move(frame))) {
    ++dropped_frames_;
    return common::Status::Unavailable("localization input queue is full");
  }
  pending_inputs_.fetch_add(1, std::memory_order_release);
  input_wait_cv_.notify_one();
  return common::Status::Ok();
}

bool LocalizationEngine::WaitForInput(std::chrono::milliseconds timeout) const {
  if (pending_inputs_.load(std::memory_order_acquire) > 0U) {
    return true;
  }
  std::unique_lock<std::mutex> lock(input_wait_mutex_);
  return input_wait_cv_.wait_for(lock, timeout, [this]() {
    return pending_inputs_.load(std::memory_order_acquire) > 0U;
  });
}

void LocalizationEngine::SetLatestOdom(const data::OdomState& odom) {
  latest_odom_.Publish(odom);
  has_latest_odom_ = true;
}

common::Status LocalizationEngine::ProcessOnce() {
  sync::SyncedFrameHandle handle;
  if (!input_queue_.try_pop(&handle)) {
    return common::Status::NotReady("no synced frame for localization");
  }
  const auto pending = pending_inputs_.load(std::memory_order_acquire);
  if (pending > 0U) {
    pending_inputs_.fetch_sub(1, std::memory_order_acq_rel);
  }

  LocalizationResult result;
  const auto status = Process(*handle.get(), &result);
  handle.reset();
  if (!status.ok()) {
    return status;
  }

  latest_result_.Publish(result);
  map_to_odom_.Publish(result.map_to_odom);
  odom_to_base_.Publish(result.odom_to_base);
  if (tf_tree_ != nullptr) {
    tf_tree_->PushMapToOdom(result.map_to_odom);
    tf_tree_->PushOdomToBase(result.odom_to_base);
  }
  return common::Status::Ok();
}

common::Status LocalizationEngine::Process(const data::SyncedFrame& frame,
                                           LocalizationResult* result) {
  if (result == nullptr) {
    return common::Status::InvalidArgument("localization result is null");
  }

  const auto begin_ns = common::NowNs();
  const LocalizationResult previous = latest_result_.ReadSnapshot();
  const data::Pose3f odom_to_base =
      has_latest_odom_ ? OdomToBaseFromState(latest_odom_.ReadSnapshot())
                       : previous.odom_to_base;
  const std::uint32_t relocalization_threshold = static_cast<std::uint32_t>(
      std::max(1, config_.relocalization_failure_threshold));
  relocalization_manager_.UpdateLockState(frame.stamp,
                                          consecutive_failures_ >= relocalization_threshold);
  result->relocalization = relocalization_manager_.CurrentStatus(frame.stamp);
  const bool coarse_relocalization_mode =
      result->relocalization.phase == RelocalizationPhase::kLostLock ||
      result->relocalization.phase == RelocalizationPhase::kSearching;
  const bool stabilization_mode =
      result->relocalization.phase == RelocalizationPhase::kStabilizing;
  const data::Pose3f predicted_map_to_base =
      coarse_relocalization_mode ? RelocalizationPrediction(frame.stamp, odom_to_base)
                                 : PredictedMapToBase(frame.stamp, odom_to_base);
  const data::Pose3f last_trusted_guess =
      last_trusted_map_to_base_.is_valid ? last_trusted_map_to_base_
                                         : RelocalizationPrediction(frame.stamp, odom_to_base);

  ScanMatchResult match;
  common::Status match_status = common::Status::NotReady("scan matcher not configured");
  const bool light_match_mode =
      !coarse_relocalization_mode && !stabilization_mode &&
      last_matcher_latency_ns_ >
      static_cast<common::TimeNs>(config_.slow_match_threshold_ms) * 1000000LL;
  result->light_match_mode = light_match_mode;
  if (config_.enabled && static_map_.global_map_loaded && matcher_ != nullptr) {
    if (coarse_relocalization_mode) {
      const auto coarse_fallback_guess = initial_pose_provider_.MakeInitialPose(frame.stamp);
      auto status = relocalization_manager_.Attempt(
          static_map_, frame.lidar, predicted_map_to_base, last_trusted_guess,
          coarse_fallback_guess, frame.stamp, &match, &result->relocalization);
      if (!status.ok()) {
        return status;
      }
      result->matcher_latency_ns = result->relocalization.attempt_latency_ns;
      last_matcher_latency_ns_ = result->matcher_latency_ns;
      current_matcher_label_ =
          std::string("relocal_") + std::string(result->relocalization.matcher_used);
      if (result->relocalization.attempted && result->relocalization.succeeded) {
        match_status = common::Status::Ok();
      } else if (result->relocalization.attempted) {
        match_status = common::Status::Unavailable("coarse relocalization did not converge");
      } else {
        match_status = common::Status::NotReady("relocalization cooldown");
      }
    } else {
      auto status = ConfigureMatcherForLoad(light_match_mode);
      if (!status.ok()) {
        return status;
      }
      const auto matcher_begin_ns = common::NowNs();
      const auto scan = light_match_mode ? DownsampleScan(frame.lidar, config_.reduced_scan_stride)
                                         : frame.lidar;
      match_status = matcher_->Match(static_map_, scan, predicted_map_to_base, &match);
      result->matcher_latency_ns = common::NowNs() - matcher_begin_ns;
      last_matcher_latency_ns_ = result->matcher_latency_ns;
      result->relocalization = relocalization_manager_.CurrentStatus(frame.stamp);
      current_matcher_label_ = config_.matcher + std::string(light_match_mode ? "_light" : "");
    }
  } else {
    result->relocalization = relocalization_manager_.CurrentStatus(frame.stamp);
    current_matcher_label_ = "disabled";
  }

  if (match_status.ok() && match.converged) {
    consecutive_failures_ = 0;
  } else {
    ++consecutive_failures_;
    match = {};
    match.matched_pose = predicted_map_to_base;
    match.matched_pose.stamp = frame.stamp;
    match.score = 0.0F;
    match.iterations = 0;
    match.converged = false;
  }

  result->odom_to_base = odom_to_base;
  result->map_to_base = match.matched_pose;
  result->map_to_base.reference_frame = tf::kMapFrame;
  result->map_to_base.child_frame = tf::kBaseLinkFrame;
  result->map_to_base.stamp = frame.stamp;
  result->map_to_base.is_valid = true;
  const data::Pose3f matched_map_to_base = result->map_to_base;

  auto status =
      map_odom_fuser_.Fuse(result->odom_to_base, result->map_to_base, &result->map_to_odom);
  if (!status.ok()) {
    return status;
  }
  result->map_to_odom.stamp = frame.stamp;

  const data::Pose3f quality_reference_pose =
      coarse_relocalization_mode ? predicted_map_to_base : previous.map_to_base;
  const bool relocalization_match_succeeded =
      result->relocalization.attempted && result->relocalization.succeeded;
  result->status = quality_estimator_.Evaluate(quality_reference_pose, match,
                                               consecutive_failures_,
                                               static_map_.global_map_loaded,
                                               relocalization_match_succeeded);
  if (coarse_relocalization_mode) {
    result->status.degraded_mode = "relocalization_search";
  } else if (stabilization_mode) {
    result->status.degraded_mode = "relocalization_stabilizing";
  } else if (light_match_mode) {
    result->status.degraded_mode = "light_match";
  }
  if (relocalization_match_succeeded) {
    relocalization_manager_.SetPendingMapToOdomTarget(result->map_to_odom);
  }
  if (relocalization_manager_.has_pending_map_to_odom_target()) {
    const data::Pose3f map_to_odom_reference =
        previous.map_to_odom.is_valid ? previous.map_to_odom : result->map_to_odom;
    result->map_to_odom = relocalization_manager_.BlendMapToOdomTarget(map_to_odom_reference);
    result->map_to_odom.stamp = frame.stamp;
    result->map_to_base = tf::Compose(result->map_to_odom, result->odom_to_base);
    result->map_to_base.reference_frame = tf::kMapFrame;
    result->map_to_base.child_frame = tf::kBaseLinkFrame;
    result->map_to_base.stamp = frame.stamp;
    result->map_to_base.is_valid = true;
  }

  const bool bypass_map_to_odom_guard =
      relocalization_match_succeeded || relocalization_manager_.stabilizing() ||
      relocalization_manager_.has_pending_map_to_odom_target();
  if (previous.map_to_odom.is_valid &&
      !bypass_map_to_odom_guard &&
      !IsMapToOdomUpdateStable(previous.map_to_odom, result->map_to_odom)) {
    const float guard_dx = result->map_to_odom.position.x - previous.map_to_odom.position.x;
    const float guard_dy = result->map_to_odom.position.y - previous.map_to_odom.position.y;
    result->status.map_to_odom_guard_translation_m =
        std::sqrt(guard_dx * guard_dx + guard_dy * guard_dy);
    result->status.map_to_odom_guard_yaw_rad = std::fabs(
        NormalizeAngle(result->map_to_odom.rpy.z - previous.map_to_odom.rpy.z));
    result->status.map_to_odom_guard_failed_translation =
        result->status.map_to_odom_guard_translation_m >
        static_cast<float>(config_.map_to_odom_guard_translation_m);
    result->status.map_to_odom_guard_failed_yaw =
        result->status.map_to_odom_guard_yaw_rad >
        static_cast<float>(config_.map_to_odom_guard_yaw_rad);
    result->map_to_odom = previous.map_to_odom;
    result->map_to_odom.stamp = frame.stamp;
    result->map_to_base = tf::Compose(result->map_to_odom, result->odom_to_base);
    result->map_to_base.stamp = frame.stamp;
    result->status.pose_trusted = false;
    result->status.converged = false;
    result->status.consecutive_failures = ++consecutive_failures_;
    result->status.rejection_reason = "map_to_odom_guard";
    result->status.degraded_mode = "map_to_odom_guard";
  }

  if (relocalization_manager_.stabilizing()) {
    relocalization_manager_.RecordStabilizationObservation(frame.stamp, matched_map_to_base,
                                                           result->status.pose_trusted,
                                                           &result->relocalization);
    if (result->relocalization.stabilization_failed) {
      const data::Pose3f fallback_map_to_odom =
          has_trusted_map_to_odom_ && last_trusted_map_to_odom_.is_valid
              ? last_trusted_map_to_odom_
              : (previous.map_to_odom.is_valid ? previous.map_to_odom : result->map_to_odom);
      result->map_to_odom = fallback_map_to_odom;
      result->map_to_odom.stamp = frame.stamp;
      result->map_to_base = tf::Compose(result->map_to_odom, result->odom_to_base);
      result->map_to_base.reference_frame = tf::kMapFrame;
      result->map_to_base.child_frame = tf::kBaseLinkFrame;
      result->map_to_base.stamp = frame.stamp;
      result->map_to_base.is_valid = true;
      result->status.pose_trusted = false;
      result->status.converged = false;
      result->status.rejection_reason = "relocalization_stabilization_failed";
      result->status.degraded_mode = "relocalization_search";
    }
    if (relocalization_manager_.stabilizing()) {
      result->status.pose_trusted = false;
      if (result->status.rejection_reason == "none") {
        result->status.rejection_reason = "stabilization_window";
      }
    }
  } else {
    result->relocalization = relocalization_manager_.CurrentStatus(frame.stamp);
  }

  result->matcher_name = current_matcher_label_;
  result->status.source = result->matcher_name;
  result->status.lost_lock = result->relocalization.phase != RelocalizationPhase::kTracking;

  if (result->status.pose_trusted) {
    last_trusted_map_to_base_ = result->map_to_base;
    last_trusted_map_to_odom_ = result->map_to_odom;
    has_trusted_map_to_odom_ = true;
    last_good_stamp_ = frame.stamp;
    consecutive_rejections_ = 0;
    relocalization_manager_.SetLastTrustedPose(result->map_to_base);
  } else {
    ++consecutive_rejections_;
    IncrementRejectionCounter(result->status.rejection_reason, &rejection_counters_);
  }
  result->status.last_good_stamp_ns = common::ToNanoseconds(last_good_stamp_);
  result->status.consecutive_rejections = consecutive_rejections_;
  result->status.rejection_counters = rejection_counters_;
  result->processing_latency_ns = common::NowNs() - begin_ns;

  const data::LidarFrame aligned_scan = TransformScanToMap(frame.lidar, result->map_to_base);
  latest_aligned_scan_.Publish(aligned_scan);
  ++processed_frames_;

  const bool should_log_state =
      current_matcher_label_ != last_logged_matcher_label_ ||
      result->relocalization.phase != last_logged_relocalization_phase_ ||
      result->status.pose_trusted != last_logged_pose_trusted_ ||
      result->status.rejection_reason != last_logged_rejection_reason_ ||
      result->light_match_mode != last_logged_light_match_mode_ ||
      result->relocalization.attempted;
  if (should_log_state) {
    std::ostringstream stream;
    stream << "mode=" << ToString(result->relocalization.phase)
           << " map=" << static_map_.version_label
           << " matcher=" << result->matcher_name
           << " score=" << result->status.match_score
           << " trusted=" << (result->status.pose_trusted ? "true" : "false")
           << " reject=" << result->status.rejection_reason
           << " degraded=" << result->status.degraded_mode
           << " relocal=" << (result->relocalization.succeeded
                                  ? "success"
                                  : (result->relocalization.attempted ? "failed" : "idle"))
           << " relocal_matcher=" << result->relocalization.matcher_used
           << " relocal_score=" << result->relocalization.best_score;
    if (result->relocalization.ambiguity_rejected) {
      stream << " ambiguity_rejected=true";
    }
    if (result->relocalization.stabilization_failed) {
      stream << " stabilization_failed=true";
    }
    if (result->status.pose_trusted) {
      utils::LogInfo("localization", stream.str());
    } else {
      utils::LogWarn("localization", stream.str());
    }
    last_logged_matcher_label_ = current_matcher_label_;
    last_logged_relocalization_phase_ = result->relocalization.phase;
    last_logged_pose_trusted_ = result->status.pose_trusted;
    last_logged_rejection_reason_ = result->status.rejection_reason;
    last_logged_light_match_mode_ = result->light_match_mode;
  }
  return common::Status::Ok();
}

data::LidarFrame LocalizationEngine::DownsampleScan(const data::LidarFrame& scan,
                                                    int stride) const {
  if (stride <= 1 || scan.points.size() <= 2U) {
    return scan;
  }
  data::LidarFrame downsampled = scan;
  downsampled.points.clear();
  downsampled.points.reserve((scan.points.size() + static_cast<std::size_t>(stride) - 1U) /
                             static_cast<std::size_t>(stride));
  for (std::size_t index = 0; index < scan.points.size(); index += static_cast<std::size_t>(stride)) {
    downsampled.points.push_back(scan.points[index]);
  }
  return downsampled;
}

data::Pose3f LocalizationEngine::OdomToBaseFromState(const data::OdomState& odom) const {
  data::Pose3f pose;
  pose.stamp = odom.stamp;
  pose.reference_frame = tf::kOdomFrame;
  pose.child_frame = tf::kBaseLinkFrame;
  pose.position.x = odom.x_m;
  pose.position.y = odom.y_m;
  pose.rpy.z = odom.yaw_rad;
  pose.is_valid = true;
  return pose;
}

data::Pose3f LocalizationEngine::PredictedMapToBase(common::TimePoint stamp,
                                                    const data::Pose3f& odom_to_base) const {
  const LocalizationResult previous = latest_result_.ReadSnapshot();
  if (processed_frames_ == 0U) {
    return initial_pose_provider_.MakeInitialPose(stamp);
  }
  if (previous.map_to_odom.is_valid && odom_to_base.is_valid) {
    data::Pose3f predicted = tf::Compose(previous.map_to_odom, odom_to_base);
    predicted.stamp = stamp;
    predicted.reference_frame = tf::kMapFrame;
    predicted.child_frame = tf::kBaseLinkFrame;
    predicted.rpy.z = NormalizeAngle(predicted.rpy.z);
    predicted.is_valid = true;
    return predicted;
  }
  return initial_pose_provider_.MakeInitialPose(stamp);
}

data::Pose3f LocalizationEngine::RelocalizationPrediction(common::TimePoint stamp,
                                                          const data::Pose3f& odom_to_base) const {
  if (has_trusted_map_to_odom_ && last_trusted_map_to_odom_.is_valid && odom_to_base.is_valid) {
    data::Pose3f predicted = tf::Compose(last_trusted_map_to_odom_, odom_to_base);
    predicted.stamp = stamp;
    predicted.reference_frame = tf::kMapFrame;
    predicted.child_frame = tf::kBaseLinkFrame;
    predicted.rpy.z = NormalizeAngle(predicted.rpy.z);
    predicted.is_valid = true;
    return predicted;
  }
  return initial_pose_provider_.MakeInitialPose(stamp);
}

bool LocalizationEngine::IsMapToOdomUpdateStable(const data::Pose3f& previous_map_to_odom,
                                                 const data::Pose3f& candidate_map_to_odom) const {
  if (!previous_map_to_odom.is_valid || !candidate_map_to_odom.is_valid) {
    return true;
  }
  const float dx = candidate_map_to_odom.position.x - previous_map_to_odom.position.x;
  const float dy = candidate_map_to_odom.position.y - previous_map_to_odom.position.y;
  const float translation = std::sqrt(dx * dx + dy * dy);
  const float yaw_delta =
      std::fabs(NormalizeAngle(candidate_map_to_odom.rpy.z - previous_map_to_odom.rpy.z));
  return translation <= static_cast<float>(config_.map_to_odom_guard_translation_m) &&
         yaw_delta <= static_cast<float>(config_.map_to_odom_guard_yaw_rad);
}

data::LidarFrame LocalizationEngine::TransformScanToMap(const data::LidarFrame& scan,
                                                        const data::Pose3f& map_to_base) const {
  data::LidarFrame transformed = scan;
  transformed.frame_id = tf::kMapFrame;
  transformed.points.clear();
  transformed.points.reserve(scan.points.size());
  for (const auto& point : scan.points) {
    transformed.points.push_back(TransformPoint(point, map_to_base));
  }
  return transformed;
}

}  // namespace rm_nav::localization
