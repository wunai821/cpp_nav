#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::planning {

class CenterHoldController {
 public:
  explicit CenterHoldController(const config::PlannerConfig& config = {}) : config_(config) {}

  void Reset();
  common::Status BuildHoldCommand(const data::Pose3f& current_pose,
                                  const data::Pose3f& target_pose,
                                  const data::GridMap2D& local_costmap,
                                  const std::vector<data::DynamicObstacle>& obstacles,
                                  data::Path2D* path,
                                  data::ChassisCmd* cmd);

 private:
  struct HoldCandidate {
    data::Pose3f pose{};
    int slot_index{0};
    bool occupied{false};
    float score{-1.0e9F};
  };

  std::vector<HoldCandidate> BuildCandidates(const data::Pose3f& current_pose,
                                             const data::Pose3f& target_pose,
                                             const data::GridMap2D& local_costmap,
                                             const std::vector<data::DynamicObstacle>& obstacles) const;
  data::Pose3f ApplyDynamicBias(const data::Pose3f& candidate,
                                const std::vector<data::DynamicObstacle>& obstacles) const;

  config::PlannerConfig config_{};
  int last_slot_index_{0};
  data::Pose3f last_hold_pose_{};
};

}  // namespace rm_nav::planning
