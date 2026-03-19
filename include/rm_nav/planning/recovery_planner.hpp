#pragma once

#include <string>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/fsm/mode_recovery.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"

namespace rm_nav::planning {

struct RecoveryPlannerInput {
  common::TimePoint stamp{};
  const data::Pose3f* current_pose{nullptr};
  const data::Path2D* global_path{nullptr};
  const data::GridMap2D* costmap{nullptr};
  const std::vector<data::DynamicObstacle>* obstacles{nullptr};
  const localization::LocalizationResult* localization_result{nullptr};
  const PlannerStatus* planner_status{nullptr};
  const data::ChassisCmd* nominal_cmd{nullptr};
};

struct RecoveryPlannerStatus {
  bool active{false};
  bool requires_relocalization{false};
  bool complete{false};
  bool exhausted{false};
  bool cooldown_active{false};
  int failure_streak{0};
  int cycles_in_tier{0};
  fsm::RecoveryTier tier{fsm::RecoveryTier::kNone};
  fsm::RecoveryCause cause{fsm::RecoveryCause::kNone};
  fsm::RecoveryAction action{fsm::RecoveryAction::kNone};
  std::string strategy{"none"};
  std::string detail{"none"};
  float clearance_weight_scale{1.0F};
  bool temporary_goal_valid{false};
  data::Pose3f temporary_goal{};
  data::ChassisCmd command{};
};

class RecoveryPlanner {
 public:
  common::Status Configure(const config::PlannerConfig& config);
  void Reset();
  common::Status Plan(const RecoveryPlannerInput& input, data::ChassisCmd* cmd,
                      RecoveryPlannerStatus* status);
  RecoveryPlannerStatus LatestStatus() const { return latest_status_; }

 private:
  fsm::RecoveryCause DetermineCause(const RecoveryPlannerInput& input) const;
  data::ChassisCmd BuildLightRecoveryCommand(const RecoveryPlannerInput& input,
                                             RecoveryPlannerStatus* status);
  data::ChassisCmd BuildMediumRecoveryCommand(const RecoveryPlannerInput& input,
                                              RecoveryPlannerStatus* status);
  data::ChassisCmd BuildHeavyRecoveryCommand(const RecoveryPlannerInput& input,
                                             RecoveryPlannerStatus* status);
  data::Pose3f BuildReapproachGoal(const RecoveryPlannerInput& input) const;
  float PreferredLateralDirection(const RecoveryPlannerInput& input) const;
  void UpdateTierState(fsm::RecoveryCause cause);

  config::PlannerConfig config_{};
  RecoveryPlannerStatus latest_status_{};
  fsm::RecoveryTier active_tier_{fsm::RecoveryTier::kNone};
  fsm::RecoveryCause active_cause_{fsm::RecoveryCause::kNone};
  int failure_streak_{0};
  int cycles_in_tier_{0};
  int cooldown_ticks_remaining_{0};
  bool configured_{false};
};

}  // namespace rm_nav::planning
