#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::planning {

class CenterHoldController {
 public:
  explicit CenterHoldController(const config::PlannerConfig& config = {}) : config_(config) {}

  common::Status BuildHoldCommand(const data::Pose3f& current_pose,
                                  const data::Pose3f& target_pose,
                                  data::Path2D* path,
                                  data::ChassisCmd* cmd) const;

 private:
  config::PlannerConfig config_{};
};

}  // namespace rm_nav::planning
