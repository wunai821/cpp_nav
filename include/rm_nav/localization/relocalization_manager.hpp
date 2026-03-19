#pragma once

#include <cstdint>
#include <vector>
#include <string_view>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/localization/ndt_matcher.hpp"

namespace rm_nav::localization {

enum class RelocalizationPhase {
  kTracking = 0,
  kLostLock,
  kSearching,
  kStabilizing,
};

enum class RelocalizationRecoveryAction {
  kNone = 0,
  kSlowSpin,
  kBackoffSpin,
  kFixedPoseSweep,
};

struct RelocalizationStatus {
  bool active{false};
  bool attempted{false};
  bool succeeded{false};
  bool ambiguity_rejected{false};
  bool stabilization_failed{false};
  bool secondary_check_performed{false};
  bool secondary_check_passed{false};
  bool map_to_odom_blending_active{false};
  bool fallback_matcher_used{false};
  int candidates_tested{0};
  int failed_attempts{0};
  int stabilization_observations{0};
  float best_score{0.0F};
  common::TimeNs attempt_latency_ns{0};
  common::TimeNs lost_lock_elapsed_ns{0};
  common::TimeNs stabilization_elapsed_ns{0};
  std::string_view matcher_used{"none"};
  RelocalizationPhase phase{RelocalizationPhase::kTracking};
  RelocalizationRecoveryAction recovery_action{RelocalizationRecoveryAction::kNone};
  data::Pose3f best_initial_guess{};
  data::Pose3f matched_pose{};
  data::ChassisCmd recovery_cmd{};
};

class RelocalizationManager {
 public:
  common::Status Configure(const config::LocalizationConfig& config);
  void Reset();
  void SetLastTrustedPose(const data::Pose3f& pose);
  void UpdateLockState(common::TimePoint stamp, bool lock_lost);
  void RecordStabilizationObservation(common::TimePoint stamp,
                                      const data::Pose3f& observed_pose,
                                      bool pose_trusted,
                                      RelocalizationStatus* status);
  bool lost_lock_active() const { return phase_ != RelocalizationPhase::kTracking; }
  bool stabilizing() const { return phase_ == RelocalizationPhase::kStabilizing; }
  bool has_pending_map_to_odom_target() const { return has_pending_map_to_odom_target_; }
  void SetPendingMapToOdomTarget(const data::Pose3f& map_to_odom);
  void ClearPendingMapToOdomTarget();
  data::Pose3f BlendMapToOdomTarget(const data::Pose3f& current_map_to_odom);
  RelocalizationStatus CurrentStatus(common::TimePoint stamp) const;
  common::Status Attempt(const StaticMap& map, const data::LidarFrame& scan,
                         const data::Pose3f& predicted_guess,
                         const data::Pose3f& last_trusted_guess,
                         const data::Pose3f& fallback_guess, common::TimePoint stamp,
                         ScanMatchResult* best_match,
                         RelocalizationStatus* status);

 private:
  struct CandidateMatch {
    ScanMatchResult match{};
    data::Pose3f initial_guess{};
    std::string_view matcher_used{"none"};
    bool fallback_used{false};
    bool valid{false};
  };

  StaticMap BuildLocalSubmap(const StaticMap& map, const data::Pose3f& center_pose) const;
  StaticMap BuildValidationSubmap(const StaticMap& map,
                                  const data::Pose3f& center_pose) const;
  std::vector<data::Pose3f> BuildCandidates(const data::Pose3f& predicted_guess,
                                            const data::Pose3f& last_trusted_guess,
                                            const data::Pose3f& fallback_guess) const;
  std::vector<data::Pose3f> BuildFixedPoseSet(const data::Pose3f& seed) const;
  bool IsAmbiguous(const CandidateMatch& best, const CandidateMatch& second_best) const;
  bool ResolveAmbiguity(const StaticMap& map, const data::LidarFrame& scan,
                        const data::Pose3f& predicted_guess,
                        const data::Pose3f& last_trusted_guess,
                        const data::Pose3f& fallback_guess, CandidateMatch* best,
                        CandidateMatch* second_best,
                        RelocalizationStatus* status) const;
  float PoseConsistencyCost(const data::Pose3f& pose, const data::Pose3f& predicted_guess,
                            const data::Pose3f& last_trusted_guess,
                            const data::Pose3f& fallback_guess) const;
  RelocalizationRecoveryAction CurrentRecoveryAction() const;
  data::ChassisCmd BuildRecoveryCommand(common::TimePoint stamp) const;
  bool IsStableObservationPose(const data::Pose3f& observed_pose) const;
  void ResetStabilizationWindow();
  void FillStatus(common::TimePoint stamp, RelocalizationStatus* status) const;

  config::LocalizationConfig config_{};
  IcpMatcher coarse_icp_matcher_{};
  NdtMatcher coarse_ndt_matcher_{};
  common::TimePoint last_attempt_stamp_{};
  common::TimePoint lost_lock_started_at_{};
  common::TimePoint stabilization_started_at_{};
  data::Pose3f last_trusted_pose_{};
  data::Pose3f pending_map_to_odom_target_{};
  data::Pose3f stabilization_anchor_pose_{};
  RelocalizationPhase phase_{RelocalizationPhase::kTracking};
  int failed_attempts_{0};
  int stabilization_observations_{0};
  int recovery_action_ticks_{0};
  bool has_last_trusted_pose_{false};
  bool has_pending_map_to_odom_target_{false};
  bool has_stabilization_anchor_pose_{false};
  bool configured_{false};
};

}  // namespace rm_nav::localization
