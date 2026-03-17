#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/planning/goal_manager.hpp"

namespace rm_nav::planning {

struct DwaScore {
  float goal_score{0.0F};
  float path_score{0.0F};
  float smooth_score{0.0F};
  float heading_score{0.0F};
  float clearance_score{0.0F};
  float velocity_score{0.0F};
  float dynamic_risk_score{0.0F};
  float dynamic_max_risk{0.0F};
  float dynamic_integrated_risk{0.0F};
  float dynamic_clearance_min{10.0F};
  float dynamic_risk_05{0.0F};
  float dynamic_risk_10{0.0F};
  float total_score{-1.0e9F};
};

class OmniDwa {
 public:
  explicit OmniDwa(const config::PlannerConfig& config) : config_(config) {}

  common::Status Plan(const data::Pose3f& current_pose,
                      const GoalState& goal,
                      const data::Path2D& global_path,
                      const data::GridMap2D& local_costmap,
                      const std::vector<data::DynamicObstacle>& obstacles,
                      const data::ChassisCmd& previous_cmd,
                      data::ChassisCmd* cmd,
                      DwaScore* score) const;

 private:
  config::PlannerConfig config_{};
};

}  // namespace rm_nav::planning
