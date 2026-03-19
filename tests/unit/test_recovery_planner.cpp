#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/planning/recovery_planner.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y) {
  rm_nav::data::Pose3f pose;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.is_valid = true;
  return pose;
}

}  // namespace

int main() {
  rm_nav::planning::RecoveryPlanner planner;
  rm_nav::config::PlannerConfig planner_config;
  planner_config.recovery_reapproach_lookahead_points = 4;
  assert(planner.Configure(planner_config).ok());

  rm_nav::data::Pose3f pose = MakePose(1.0F, 2.0F);
  rm_nav::data::Path2D path;
  for (float x : {1.0F, 1.4F, 1.8F, 2.2F, 2.6F}) {
    rm_nav::data::PathPoint2f point;
    point.position.x = x;
    point.position.y = 2.0F;
    point.heading_rad = 0.0F;
    point.target_speed_mps = 0.3F;
    path.points.push_back(point);
  }
  rm_nav::data::GridMap2D costmap;
  rm_nav::data::ChassisCmd nominal_cmd;
  nominal_cmd.vx_mps = 0.4F;
  nominal_cmd.vy_mps = 0.0F;

  rm_nav::localization::LocalizationResult localization_result;
  localization_result.map_to_base = pose;
  localization_result.status.pose_trusted = true;

  rm_nav::planning::PlannerStatus planner_status;
  planner_status.failure_reason = "local_plan_failed";
  planner_status.local_plan_succeeded = false;
  planner_status.global_plan_succeeded = true;
  planner_status.distance_to_goal_m = 2.5F;
  planner_status.dwa_score.dynamic_max_risk = 1.1F;
  planner_status.dwa_score.dynamic_crossing_penalty = 0.4F;

  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose(1.8F, 2.2F);
  obstacle.pose.is_valid = true;
  obstacle.radius_m = 0.3F;
  std::vector<rm_nav::data::DynamicObstacle> obstacles{obstacle};

  rm_nav::planning::RecoveryPlannerInput input;
  input.stamp = rm_nav::common::Now();
  input.current_pose = &pose;
  input.global_path = &path;
  input.costmap = &costmap;
  input.obstacles = &obstacles;
  input.localization_result = &localization_result;
  input.planner_status = &planner_status;
  input.nominal_cmd = &nominal_cmd;

  rm_nav::data::ChassisCmd recovery_cmd;
  rm_nav::planning::RecoveryPlannerStatus recovery_status;
  assert(planner.Plan(input, &recovery_cmd, &recovery_status).ok());
  assert(recovery_status.tier == rm_nav::fsm::RecoveryTier::kLight);
  assert(recovery_status.cause == rm_nav::fsm::RecoveryCause::kDynamicBlock);
  assert(recovery_status.action == rm_nav::fsm::RecoveryAction::kSlowdownResample);
  assert(recovery_status.strategy == "l1_dynamic_gap_release");
  assert(!recovery_cmd.brake);
  assert(std::fabs(recovery_cmd.wz_radps) > 0.0F || std::fabs(recovery_cmd.vy_mps) > 0.0F);

  planner_status.distance_to_goal_m = 0.45F;
  planner_status.dwa_score.dynamic_max_risk = 0.0F;
  planner_status.dwa_score.dynamic_crossing_penalty = 0.0F;
  for (int i = 0; i < 13; ++i) {
    assert(planner.Plan(input, &recovery_cmd, &recovery_status).ok());
  }
  assert(recovery_status.tier == rm_nav::fsm::RecoveryTier::kMedium);
  assert(recovery_status.cause == rm_nav::fsm::RecoveryCause::kStuckNearGoal);
  assert(recovery_status.action == rm_nav::fsm::RecoveryAction::kReapproachWaypoint);
  assert(recovery_status.temporary_goal_valid);
  assert(recovery_status.temporary_goal.is_valid);
  assert(recovery_status.strategy == "l2_goal_unstick");
  assert(!recovery_cmd.brake);
  assert(recovery_cmd.vx_mps >= 0.0F || std::fabs(recovery_cmd.vy_mps) > 0.0F);

  localization_result.status.pose_trusted = false;
  localization_result.status.rejection_reason = "map_to_odom_guard";
  planner_status.distance_to_goal_m = 2.0F;
  assert(planner.Plan(input, &recovery_cmd, &recovery_status).ok());
  assert(recovery_status.tier == rm_nav::fsm::RecoveryTier::kHeavy);
  assert(recovery_status.requires_relocalization);
  assert(recovery_status.action == rm_nav::fsm::RecoveryAction::kStopAndRelocalize);
  assert(recovery_cmd.brake);

  localization_result.status.pose_trusted = true;
  localization_result.status.rejection_reason = "none";
  planner_status.failure_reason = "none";
  planner_status.local_plan_succeeded = true;
  assert(planner.Plan(input, &recovery_cmd, &recovery_status).ok());
  assert(recovery_status.cooldown_active);
  assert(recovery_status.action == rm_nav::fsm::RecoveryAction::kSlowRestart);
  assert(!recovery_cmd.brake);
  assert(std::fabs(recovery_cmd.vx_mps) < nominal_cmd.vx_mps);

  std::cout << "test_recovery_planner passed\n";
  return 0;
}
