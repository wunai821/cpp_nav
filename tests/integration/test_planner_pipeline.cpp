#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::localization::StaticMap BuildMap() {
  rm_nav::localization::StaticMap map;
  map.occupancy.resolution_m = 0.1F;
  map.occupancy.width = 100U;
  map.occupancy.height = 70U;
  map.occupancy.origin = MakePose(0.0F, 0.0F, 0.0F);
  map.occupancy.origin.child_frame = rm_nav::tf::kMapFrame;
  map.occupancy.occupancy.assign(map.occupancy.width * map.occupancy.height, 0U);
  map.occupancy_loaded = true;
  map.global_map_loaded = true;
  return map;
}

}  // namespace

int main() {
  const auto map = BuildMap();

  rm_nav::config::PlannerConfig config;
  rm_nav::planning::PlannerCoordinator planner;
  assert(planner.Initialize(config, map).ok());

  rm_nav::data::Pose3f current_pose = MakePose(1.2F, 2.0F, 0.0F);
  rm_nav::data::Pose3f custom_goal = MakePose(2.0F, 5.2F, 0.0F);

  rm_nav::data::GridMap2D costmap;
  costmap.width = 80U;
  costmap.height = 80U;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);

  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose(1.8F, 2.0F, 0.0F);
  obstacle.velocity.x = 0.0F;
  obstacle.velocity.y = 0.6F;
  obstacle.radius_m = 0.3F;
  obstacle.is_confirmed = true;

  rm_nav::data::Path2D path;
  rm_nav::data::ChassisCmd cmd;
  assert(planner.PlanToGoal(current_pose, custom_goal, costmap, {obstacle}, &path, &cmd).ok());
  assert(!path.points.empty());
  assert(std::fabs(path.points.back().position.x - custom_goal.position.x) < 0.25F);
  assert(std::fabs(path.points.back().position.y - custom_goal.position.y) < 0.25F);
  assert(!cmd.brake);

  std::cout << "test_planner_pipeline passed\n";
  return 0;
}
