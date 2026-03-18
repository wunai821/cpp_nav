#pragma once

#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/mapping/map_builder_3d.hpp"
#include "rm_nav/mapping/mapping_keyframe.hpp"
#include "rm_nav/mapping/loop_candidate_detector.hpp"
#include "rm_nav/mapping/loop_closure_matcher.hpp"
#include "rm_nav/mapping/loop_correction_gate.hpp"
#include "rm_nav/mapping/map_projector_2d.hpp"
#include "rm_nav/mapping/map_save_manager.hpp"
#include "rm_nav/mapping/map_serializer.hpp"
#include "rm_nav/mapping/mapping_trajectory.hpp"

namespace rm_nav::mapping {

struct MappingResult {
  data::Pose3f map_to_base{};
  data::Pose3f external_pose{};
  data::Pose3f predicted_pose{};
  std::size_t accumulated_points{0};
  std::size_t processed_frames{0};
  common::TimeNs processing_latency_ns{0};
  std::string pose_source{"odom"};
  float frontend_score{0.0F};
  int frontend_iterations{0};
  bool frontend_converged{false};
  bool frontend_fallback{false};
  common::TimeNs frontend_latency_ns{0};
  std::size_t frontend_reference_points{0};
};

class MappingEngine {
 public:
  common::Status Initialize(const config::MappingConfig& config);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base,
                        const std::vector<data::DynamicObstacle>& known_dynamic_obstacles);
  common::Status BuildGridMap2D(data::GridMap2D* grid_map) const;
  std::vector<data::PointXYZI> GlobalPointCloud() const { return builder_.GlobalPoints(); }
  MapBuilder3D::DynamicSuppressionDebugSnapshot LatestDynamicSuppressionDebug() const {
    return builder_.LatestDynamicSuppressionDebug();
  }
  const std::vector<MappingKeyframe>& Keyframes() const { return keyframes_; }
  const std::vector<MappingTrajectorySample>& TrajectorySamples() const {
    return trajectory_samples_;
  }
  LoopClosureCandidate LatestLoopCandidate() const { return latest_loop_candidate_; }
  LoopClosureMatchResult LatestLoopMatch() const { return latest_loop_match_; }
  LoopCorrectionDecision LatestLoopCorrection() const { return latest_loop_correction_; }
  data::Pose3f AppliedPoseCorrection() const { return applied_pose_correction_; }
  MappingResult LatestResult() const { return latest_result_; }
  common::Status SaveMap(const std::string& output_dir,
                         localization::StaticMap* exported_map = nullptr,
                         MapSaveFailureKind* failure_kind = nullptr) const;

 private:
  bool ShouldCaptureKeyframe(const data::SyncedFrame& frame,
                             const data::Pose3f& map_to_base) const;
  void CaptureKeyframe(const data::SyncedFrame& frame, const data::Pose3f& map_to_base);
  common::Status ResolveMappingPose(const data::SyncedFrame& frame,
                                    const data::Pose3f& external_pose,
                                    data::Pose3f* mapping_pose,
                                    bool* pose_is_map_aligned);
  data::Pose3f ApplyCorrection(const data::Pose3f& raw_pose) const;
  void AdvanceCorrectionTowardTarget();
  void UpdateCorrectionTarget(const data::Pose3f& raw_pose, const MappingKeyframe& keyframe,
                              const LoopCorrectionDecision& decision);
  void RecordTrajectorySample(const data::SyncedFrame& frame, const data::Pose3f& external_pose,
                              const data::Pose3f& predicted_pose,
                              const data::Pose3f& optimized_pose);

  config::MappingConfig config_{};
  MapBuilder3D builder_{};
  localization::IcpMatcher frontend_icp_matcher_{};
  LoopCandidateDetector loop_candidate_detector_{};
  LoopClosureMatcher loop_closure_matcher_{};
  LoopCorrectionGate loop_correction_gate_{};
  MapProjector2D projector_{};
  MapSaveManager save_manager_{};
  MappingResult latest_result_{};
  std::vector<MappingKeyframe> keyframes_{};
  std::vector<MappingTrajectorySample> trajectory_samples_{};
  LoopClosureCandidate latest_loop_candidate_{};
  LoopClosureMatchResult latest_loop_match_{};
  LoopCorrectionDecision latest_loop_correction_{};
  int consecutive_loop_correction_failures_{0};
  data::Pose3f previous_external_pose_{};
  data::Pose3f previous_mapping_pose_{};
  data::LidarFrame previous_scan_{};
  bool has_previous_frontend_state_{false};
  data::Pose3f applied_pose_correction_{};
  data::Pose3f target_pose_correction_{};
  bool initialized_{false};
};

}  // namespace rm_nav::mapping
