#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/safety/collision_checker.hpp"

namespace {

rm_nav::data::Pose3f MakePose() {
  rm_nav::data::Pose3f pose;
  pose.stamp = rm_nav::common::Now();
  pose.position.x = 1.0F;
  pose.position.y = 2.0F;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::GridMap2D MakeCostmap() {
  rm_nav::data::GridMap2D costmap;
  costmap.width = 21U;
  costmap.height = 21U;
  costmap.resolution_m = 0.05F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);
  return costmap;
}

void MarkOccupied(rm_nav::data::GridMap2D* costmap, float local_x, float local_y) {
  assert(costmap != nullptr);
  const int center_x = static_cast<int>(costmap->width / 2U);
  const int center_y = static_cast<int>(costmap->height / 2U);
  const int gx = center_x + static_cast<int>(std::round(local_x / costmap->resolution_m));
  const int gy = center_y + static_cast<int>(std::round(local_y / costmap->resolution_m));
  assert(gx >= 0 && gy >= 0);
  assert(gx < static_cast<int>(costmap->width) && gy < static_cast<int>(costmap->height));
  costmap->occupancy[static_cast<std::size_t>(gy) * costmap->width +
                     static_cast<std::size_t>(gx)] = 100U;
}

rm_nav::data::ChassisCmd MakeCmd(float vx, float vy, float wz) {
  rm_nav::data::ChassisCmd cmd;
  cmd.stamp = rm_nav::common::Now();
  cmd.vx_mps = vx;
  cmd.vy_mps = vy;
  cmd.wz_radps = wz;
  cmd.brake = false;
  return cmd;
}

rm_nav::data::DynamicObstacle MakeObstacle(float x, float y, float radius_m) {
  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose();
  obstacle.pose.position.x = x;
  obstacle.pose.position.y = y;
  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.radius_m = radius_m;
  obstacle.confidence = 0.95F;
  return obstacle;
}

}  // namespace

int main() {
  rm_nav::safety::CollisionChecker checker;
  rm_nav::config::SafetyConfig config;
  config.collision_check_dt_s = 0.1;
  config.collision_check_lookahead_s = 0.1;
  config.emergency_stop_distance_m = 0.1;
  config.footprint_half_length_m = 0.30;
  config.footprint_half_width_m = 0.25;

  const auto pose = MakePose();

  {
    auto costmap = MakeCostmap();
    MarkOccupied(&costmap, 0.32F, 0.25F);
    const auto result = checker.Evaluate(pose, costmap, {}, MakeCmd(0.2F, 0.0F, 0.0F), config);
    assert(result.type == rm_nav::safety::CollisionType::kStatic);
  }

  {
    auto costmap = MakeCostmap();
    const std::vector<rm_nav::data::DynamicObstacle> obstacles{
        MakeObstacle(pose.position.x, pose.position.y + 0.35F, 0.05F)};
    const auto result = checker.Evaluate(pose, costmap, obstacles, MakeCmd(0.0F, 0.0F, 0.0F),
                                         config);
    assert(result.type == rm_nav::safety::CollisionType::kDynamic);
    assert(result.obstacle_too_close);
  }

  std::cout << "test_collision_checker passed\n";
  return 0;
}
