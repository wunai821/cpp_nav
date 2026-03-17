#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/planning/goal_manager.hpp"
#include "rm_nav/planning/omni_dwa.hpp"

int main() {
  rm_nav::config::PlannerConfig config;
  rm_nav::planning::OmniDwa dwa(config);

  rm_nav::data::Pose3f pose;
  pose.position.x = 0.0F;
  pose.position.y = 0.0F;
  pose.is_valid = true;

  rm_nav::planning::GoalState goal;
  goal.target_pose.position.x = 3.0F;
  goal.target_pose.position.y = 0.0F;
  goal.target_pose.is_valid = true;
  goal.distance_to_target_m = 3.0F;

  rm_nav::data::Path2D path;
  for (int index = 0; index < 6; ++index) {
    rm_nav::data::PathPoint2f point;
    point.position.x = 0.5F * static_cast<float>(index);
    point.position.y = 0.0F;
    path.points.push_back(point);
  }

  rm_nav::data::GridMap2D costmap;
  costmap.width = 80;
  costmap.height = 80;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);

  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose.position.x = 1.0F;
  obstacle.pose.position.y = 0.0F;
  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.radius_m = 0.3F;
  obstacle.confidence = 1.0F;
  obstacle.is_confirmed = true;

  rm_nav::data::ChassisCmd cmd;
  rm_nav::planning::DwaScore score;
  assert(dwa.Plan(pose, goal, path, costmap, {obstacle}, {}, &cmd, &score).ok());
  assert(score.dynamic_risk_score < 0.0F);
  assert(std::fabs(cmd.vy_mps) > 0.01F || std::fabs(cmd.wz_radps) > 0.01F ||
         cmd.vx_mps < 0.9F);

  obstacle.pose.position.x = 4.0F;
  obstacle.predicted_pose_05s.position.x = 4.0F;
  obstacle.predicted_pose_10s.position.x = 4.0F;
  rm_nav::data::ChassisCmd far_cmd;
  rm_nav::planning::DwaScore far_score;
  assert(dwa.Plan(pose, goal, path, costmap, {obstacle}, {}, &far_cmd, &far_score).ok());
  assert(std::fabs(far_score.dynamic_risk_score) < std::fabs(score.dynamic_risk_score));
  assert(far_cmd.vx_mps >= cmd.vx_mps - 1.0e-3F);

  std::cout << "test_dwa_dynamic_risk passed\n";
  return 0;
}
