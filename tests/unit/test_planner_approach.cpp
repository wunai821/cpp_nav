#include <cassert>
#include <cmath>
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
  rm_nav::planning::PlannerCoordinator planner;
  assert(planner.Initialize(planner_config, map).ok());

  rm_nav::data::GridMap2D costmap;
  costmap.width = 80;
  costmap.height = 80;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);

  rm_nav::data::Pose3f pose;
  pose.position.x = 1.2F;
  pose.position.y = 2.0F;
  pose.rpy.z = 0.0F;
  pose.is_valid = true;

  rm_nav::data::Path2D path;
  rm_nav::data::ChassisCmd cmd;
  assert(planner.Plan(pose, costmap, {}, &path, &cmd).ok());
  assert(!path.points.empty());
  assert(!cmd.brake);
  assert(std::fabs(cmd.vx_mps) + std::fabs(cmd.vy_mps) + std::fabs(cmd.wz_radps) > 0.01F);

  std::cout << "test_planner_approach passed\n";
  return 0;
}
