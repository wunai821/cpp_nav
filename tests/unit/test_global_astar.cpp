#include <cassert>
#include <iostream>

#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/planning/global_astar.hpp"

int main() {
  rm_nav::config::LocalizationConfig localization_config;
  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;
  assert(loader.Load("config", localization_config, &map).ok());

  rm_nav::data::Pose3f start;
  start.position.x = 1.2F;
  start.position.y = 2.0F;
  start.is_valid = true;
  rm_nav::data::Pose3f goal;
  goal.position.x = 6.0F;
  goal.position.y = 3.0F;
  goal.is_valid = true;

  rm_nav::planning::GlobalAStar astar;
  rm_nav::data::Path2D path;
  assert(astar.Plan(map.occupancy, start, goal, &path).ok());
  assert(path.points.size() >= 2U);
  assert(path.points.back().position.x > 5.5F);
  assert(path.points.back().position.y > 2.5F);

  std::cout << "test_global_astar passed\n";
  return 0;
}
