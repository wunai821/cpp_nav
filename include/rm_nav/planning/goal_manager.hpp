#pragma once

#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::planning {

enum class GoalMode {
  kApproachCenter = 0,
  kCenterHold,
};

struct GoalState {
  GoalMode mode{GoalMode::kApproachCenter};
  data::Pose3f target_pose{};
  float distance_to_target_m{0.0F};
  float distance_to_center_m{0.0F};
  float yaw_error_rad{0.0F};
  bool reached{false};
};

class GoalManager {
 public:
  explicit GoalManager(const config::PlannerConfig& config) : config_(config) {}

  GoalState Update(const data::Pose3f& current_pose);
  GoalState UpdateToward(const data::Pose3f& current_pose, const data::Pose3f& target_pose);

 private:
  GoalState BuildState(const data::Pose3f& current_pose, const data::Pose3f& target_pose);

  config::PlannerConfig config_{};
  bool hold_active_{false};
};

}  // namespace rm_nav::planning
