#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/planning/omni_dwa.hpp"
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

rm_nav::data::PathPoint2f MakePathPoint(float x, float y, float yaw) {
  rm_nav::data::PathPoint2f point;
  point.position.x = x;
  point.position.y = y;
  point.heading_rad = yaw;
  point.target_speed_mps = 0.8F;
  return point;
}

}  // namespace

int main() {
  rm_nav::config::PlannerConfig config;
  rm_nav::planning::OmniDwa dwa(config);

  rm_nav::planning::GoalState goal;
  goal.mode = rm_nav::planning::GoalMode::kApproachCenter;
  goal.target_pose = MakePose(4.5F, 2.5F, 0.0F);
  goal.distance_to_target_m = 3.0F;

  rm_nav::data::Pose3f current_pose = MakePose(1.5F, 2.0F, 0.0F);
  rm_nav::data::Path2D path;
  path.points.push_back(MakePathPoint(current_pose.position.x, current_pose.position.y,
                                      current_pose.rpy.z));
  path.points.push_back(MakePathPoint(2.4F, 2.1F, 0.0F));
  path.points.push_back(MakePathPoint(3.4F, 2.2F, 0.0F));
  path.points.push_back(
      MakePathPoint(goal.target_pose.position.x, goal.target_pose.position.y,
                    goal.target_pose.rpy.z));

  rm_nav::data::GridMap2D costmap;
  costmap.width = 80U;
  costmap.height = 80U;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);

  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose(2.0F, 2.8F, 0.0F);
  obstacle.predicted_pose_05s = MakePose(2.2F, 2.7F, 0.0F);
  obstacle.predicted_pose_10s = MakePose(2.4F, 2.6F, 0.0F);
  obstacle.velocity.x = 0.4F;
  obstacle.velocity.y = -0.2F;
  obstacle.radius_m = 0.3F;
  obstacle.confidence = 0.9F;
  obstacle.is_confirmed = true;

  rm_nav::data::ChassisCmd previous_cmd;
  previous_cmd.vx_mps = 0.2F;
  previous_cmd.vy_mps = 0.0F;

  rm_nav::data::ChassisCmd cmd;
  rm_nav::planning::DwaScore score;
  assert(dwa.Plan(current_pose, goal, path, costmap, {obstacle}, previous_cmd, &cmd, &score).ok());
  assert(cmd.stamp == current_pose.stamp);
  assert(!cmd.brake);
  assert(score.total_score > -1.0e8F);
  assert(score.dynamic_clearance_min > 0.0F);
  assert(score.dynamic_integrated_risk >= 0.0F);

  std::cout << "test_omni_dwa passed\n";
  return 0;
}
