#pragma once

#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <string>

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/ring_queue.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/initial_pose_provider.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/localization/map_odom_fuser.hpp"
#include "rm_nav/localization/ndt_matcher.hpp"
#include "rm_nav/localization/pose_quality_estimator.hpp"
#include "rm_nav/localization/relocalization_manager.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"

namespace rm_nav::localization {

struct LocalizationResult {
  data::Pose3f map_to_odom{};
  data::Pose3f map_to_base{};
  data::Pose3f odom_to_base{};
  LocalizationStatus status{};
  RelocalizationStatus relocalization{};
  common::TimeNs matcher_latency_ns{0};
  common::TimeNs processing_latency_ns{0};
  bool light_match_mode{false};
  std::string matcher_name{"none"};
};

class LocalizationEngine {
 public:
  common::Status Initialize(const std::string& config_dir,
                            const config::LocalizationConfig& localization_config,
                            const config::SpawnConfig& spawn_config,
                            tf::TfTreeLite* tf_tree);
  common::Status EnqueueFrame(sync::SyncedFrameHandle frame);
  bool WaitForInput(std::chrono::milliseconds timeout) const;
  common::Status ProcessOnce();
  common::Status Process(const data::SyncedFrame& frame,
                         LocalizationResult* result);
  void SetLatestOdom(const data::OdomState& odom);

  LocalizationResult LatestResult() const { return latest_result_.ReadSnapshot(); }
  data::Pose3f LatestMapToOdom() const { return map_to_odom_.ReadSnapshot(); }
  data::Pose3f LatestOdomToBase() const { return odom_to_base_.ReadSnapshot(); }
  data::LidarFrame LatestAlignedScan() const { return latest_aligned_scan_.ReadSnapshot(); }
  const StaticMap& static_map() const { return static_map_; }
  bool map_loaded() const { return static_map_.global_map_loaded; }
  std::uint64_t dropped_frames() const { return dropped_frames_; }
  const std::string& CurrentMatcherLabel() const { return current_matcher_label_; }

 private:
  data::Pose3f OdomToBaseFromState(const data::OdomState& odom) const;
  data::Pose3f PredictedMapToBase(common::TimePoint stamp,
                                  const data::Pose3f& odom_to_base) const;
  data::LidarFrame TransformScanToMap(const data::LidarFrame& scan,
                                      const data::Pose3f& map_to_base) const;
  data::LidarFrame DownsampleScan(const data::LidarFrame& scan, int stride) const;
  common::Status ConfigureMatcherForLoad(bool light_mode);
  data::Pose3f RelocalizationPrediction(common::TimePoint stamp,
                                        const data::Pose3f& odom_to_base) const;
  bool IsMapToOdomUpdateStable(const data::Pose3f& previous_map_to_odom,
                               const data::Pose3f& candidate_map_to_odom) const;

  config::LocalizationConfig config_{};
  tf::TfTreeLite* tf_tree_{nullptr};
  common::SpscRingQueue<sync::SyncedFrameHandle, 16> input_queue_{};
  mutable std::mutex input_wait_mutex_{};
  mutable std::condition_variable input_wait_cv_{};
  std::atomic<std::uint32_t> pending_inputs_{0};
  common::DoubleBuffer<LocalizationResult> latest_result_{};
  common::DoubleBuffer<data::Pose3f> map_to_odom_{};
  common::DoubleBuffer<data::Pose3f> odom_to_base_{};
  common::DoubleBuffer<data::LidarFrame> latest_aligned_scan_{};
  common::DoubleBuffer<data::OdomState> latest_odom_{};
  StaticMap static_map_{};
  IcpMatcher icp_matcher_{};
  NdtMatcher ndt_matcher_{};
  ScanMatcher* matcher_{nullptr};
  RelocalizationManager relocalization_manager_{};
  InitialPoseProvider initial_pose_provider_{{}};
  MapOdomFuser map_odom_fuser_{};
  PoseQualityEstimator quality_estimator_{{}};
  std::uint64_t processed_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint32_t consecutive_failures_{0};
  common::TimeNs last_matcher_latency_ns_{0};
  bool matcher_light_mode_active_{false};
  bool has_latest_odom_{false};
  data::Pose3f last_trusted_map_to_base_{};
  data::Pose3f last_trusted_map_to_odom_{};
  bool has_trusted_map_to_odom_{false};
  common::TimePoint last_good_stamp_{};
  std::uint32_t consecutive_rejections_{0};
  LocalizationRejectionCounters rejection_counters_{};
  std::string current_matcher_label_{"none"};
  std::string last_logged_matcher_label_{"none"};
  RelocalizationPhase last_logged_relocalization_phase_{RelocalizationPhase::kTracking};
  std::string last_logged_rejection_reason_{"none"};
  bool last_logged_pose_trusted_{false};
  bool last_logged_light_match_mode_{false};
};

}  // namespace rm_nav::localization
