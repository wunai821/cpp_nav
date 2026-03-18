#pragma once

#include <string_view>
#include <vector>

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/planning/center_hold_controller.hpp"
#include "rm_nav/planning/global_astar.hpp"
#include "rm_nav/planning/goal_manager.hpp"
#include "rm_nav/planning/mission_manager.hpp"
#include "rm_nav/planning/omni_dwa.hpp"

namespace rm_nav::planning {

struct PlannerStatus {
  GoalMode mode{GoalMode::kApproachCenter};
  float distance_to_goal_m{0.0F};
  float distance_to_center_m{0.0F};
  float yaw_error_rad{0.0F};
  bool reached{false};
  bool settling{false};
  bool hold_drifted{false};
  bool path_available{false};
  int hold_frames_in_goal{0};
  common::TimeNs hold_settle_elapsed_ns{0};
  DwaScore dwa_score{};
  common::TimeNs planning_latency_ns{0};
};

class PlannerCoordinator {
 public:
  common::Status Initialize(const config::PlannerConfig& config,
                            const localization::StaticMap& static_map);
  common::Status Plan(
      const data::Pose3f& current_pose, const data::GridMap2D& costmap,
      const std::vector<data::DynamicObstacle>& obstacles, data::Path2D* path,
      data::ChassisCmd* cmd);
  common::Status PlanToGoal(
      const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
      const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles,
      data::Path2D* path, data::ChassisCmd* cmd);
  common::Status PlanAndPublish(const data::Pose3f& current_pose,
                                const data::GridMap2D& costmap,
                                const std::vector<data::DynamicObstacle>& obstacles);
  common::Status PlanAndPublishToGoal(
      const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
      const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles);
  data::Path2D LatestPath() const { return global_path_.ReadSnapshot(); }
  data::ChassisCmd LatestCmd() const { return latest_cmd_.ReadSnapshot(); }
  PlannerStatus LatestStatus() const { return latest_status_.ReadSnapshot(); }

 private:
  config::PlannerConfig config_{};
  localization::StaticMap static_map_{};
  GoalManager goal_manager_{{}};
  MissionManager mission_manager_{{}};
  CenterHoldController center_hold_controller_{{}};
  GlobalAStar global_astar_{};
  OmniDwa omni_dwa_{{}};
  common::DoubleBuffer<data::Path2D> global_path_{};
  common::DoubleBuffer<data::ChassisCmd> latest_cmd_{};
  common::DoubleBuffer<PlannerStatus> latest_status_{};
  bool initialized_{false};
};

}  // namespace rm_nav::planning
