#include <cassert>
#include <chrono>
#include <iostream>

#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"

int main() {
  rm_nav::config::LocalizationConfig localization_config;
  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;
  assert(loader.Load("config", localization_config, &map).ok());

  rm_nav::config::PlannerConfig planner_config;
  planner_config.center_hold_settle_frames = 3;
  planner_config.center_hold_settle_time_ms = 120;
  rm_nav::planning::PlannerCoordinator planner;
  assert(planner.Initialize(planner_config, map).ok());

  rm_nav::data::GridMap2D costmap;
  costmap.width = 40;
  costmap.height = 40;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);

  rm_nav::data::Pose3f near_center;
  const auto start = rm_nav::common::Now();
  near_center.stamp = start;
  near_center.position.x = 6.02F;
  near_center.position.y = 3.03F;
  near_center.rpy.z = 0.03F;
  near_center.is_valid = true;

  rm_nav::data::Path2D path;
  rm_nav::data::ChassisCmd cmd;
  assert(planner.Plan(near_center, costmap, {}, &path, &cmd).ok());
  auto status = planner.LatestStatus();
  assert(status.mode == rm_nav::planning::GoalMode::kApproachCenter);
  assert(!status.reached);

  near_center.stamp = start + std::chrono::milliseconds(60);
  assert(planner.Plan(near_center, costmap, {}, &path, &cmd).ok());
  status = planner.LatestStatus();
  assert(!status.reached);

  near_center.stamp = start + std::chrono::milliseconds(140);
  assert(planner.Plan(near_center, costmap, {}, &path, &cmd).ok());
  status = planner.LatestStatus();
  assert(status.mode == rm_nav::planning::GoalMode::kCenterHold);
  assert(status.reached);
  assert(status.hold_frames_in_goal >= 3);
  assert(status.hold_settle_elapsed_ns > 0);
  assert(cmd.brake);
  assert(cmd.vx_mps == 0.0F);
  assert(cmd.vy_mps == 0.0F);
  assert(cmd.wz_radps == 0.0F);
  assert(path.points.size() == 1U);

  std::cout << "test_center_hold passed\n";
  return 0;
}
