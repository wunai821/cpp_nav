#pragma once

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/planning/goal_manager.hpp"

namespace rm_nav::planning {

enum class MissionPhase {
  kApproach = 0,
  kSettling,
  kCenterHold,
};

struct MissionStatus {
  MissionPhase phase{MissionPhase::kApproach};
  GoalMode mode{GoalMode::kApproachCenter};
  bool reached{false};
  bool settling{false};
  bool hold_drifted{false};
  int consecutive_in_goal_frames{0};
  common::TimeNs settle_elapsed_ns{0};
};

class MissionManager {
 public:
  explicit MissionManager(const config::PlannerConfig& config = {}) : config_(config) {}

  void Reset();
  MissionStatus Update(const data::Pose3f& current_pose, const GoalState& goal);

 private:
  bool TargetChanged(const data::Pose3f& target_pose) const;
  void ResetSettleTracking();

  config::PlannerConfig config_{};
  MissionPhase phase_{MissionPhase::kApproach};
  data::Pose3f tracked_target_{};
  int consecutive_in_goal_frames_{0};
  common::TimePoint in_goal_since_{};
};

}  // namespace rm_nav::planning
