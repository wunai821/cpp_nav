#pragma once

#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/referee_state.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"

namespace rm_nav::app {

enum class MatchModePhase {
  kCenter = 0,
  kReturnToSpawn,
  kWaitingAtSpawn,
  kReturnToCenter,
};

struct MatchModeConfig {
  bool enabled{false};
  int low_hp_threshold{100};
  int spawn_wait_ms{5000};
  float spawn_reach_tolerance_m{0.6F};
  data::Pose3f spawn_pose{};
  float center_reach_tolerance_m{0.6F};
  data::Pose3f center_pose{};
};

struct MatchModeDecision {
  MatchModePhase phase{MatchModePhase::kCenter};
  bool override_goal{false};
  bool count_as_center_goal{true};
  bool rearm_required{false};
  data::Pose3f goal_pose{};
  common::TimeNs wait_remaining_ns{0};
  std::string reason{"none"};
};

class MatchModeController {
 public:
  common::Status Configure(const MatchModeConfig& config);
  void Reset();
  MatchModeDecision Update(common::TimePoint stamp, const data::RefereeState& referee,
                           const data::Pose3f& current_pose,
                           const planning::PlannerStatus& planner_status);
  MatchModeDecision LatestDecision() const { return latest_decision_; }

 private:
  bool ResourcesLow(const data::RefereeState& referee) const;
  bool ResourcesRecovered(const data::RefereeState& referee) const;
  bool NearSpawn(const data::Pose3f& current_pose) const;
  bool NearCenter(const data::Pose3f& current_pose) const;

  MatchModeConfig config_{};
  MatchModeDecision latest_decision_{};
  common::TimePoint wait_deadline_{};
  bool configured_{false};
};

const char* ToString(MatchModePhase phase);

}  // namespace rm_nav::app
