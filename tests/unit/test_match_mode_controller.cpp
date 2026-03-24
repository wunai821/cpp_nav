#include <cassert>

#include "rm_nav/app/match_mode_controller.hpp"

int main() {
  rm_nav::app::MatchModeController controller;
  rm_nav::app::MatchModeConfig config;
  config.enabled = true;
  config.low_hp_threshold = 100;
  config.spawn_wait_ms = 1000;
  config.spawn_reach_tolerance_m = 0.5F;
  config.spawn_pose.is_valid = true;
  config.spawn_pose.position.x = 1.0F;
  config.spawn_pose.position.y = 2.0F;
  config.center_reach_tolerance_m = 0.6F;
  config.center_pose.is_valid = true;
  config.center_pose.position.x = 6.0F;
  config.center_pose.position.y = 3.0F;
  assert(controller.Configure(config).ok());

  rm_nav::data::RefereeState referee;
  referee.is_online = true;
  referee.game_stage = 2;
  referee.remaining_time_s = 180;
  referee.robot_hp = 200;
  referee.ammo = 10;

  rm_nav::data::Pose3f pose;
  pose.is_valid = true;
  pose.position.x = 5.0F;
  pose.position.y = 3.0F;
  pose.stamp = rm_nav::common::Now();

  rm_nav::planning::PlannerStatus planner_status;
  auto decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kCenter);
  assert(!decision.override_goal);
  assert(decision.count_as_center_goal);

  referee.ammo = 0;
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kReturnToSpawn);
  assert(decision.override_goal);
  assert(!decision.count_as_center_goal);
  assert(decision.reason == "ammo_empty");

  pose.position.x = 1.0F;
  pose.position.y = 2.0F;
  pose.stamp += std::chrono::milliseconds(100);
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kWaitingAtSpawn);
  assert(decision.override_goal);
  assert(decision.wait_remaining_ns > 0);

  pose.stamp += std::chrono::milliseconds(1200);
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kReturnToCenter);
  assert(!decision.override_goal);
  assert(decision.count_as_center_goal);
  assert(decision.rearm_required);

  planner_status.reached = true;
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kReturnToCenter);

  pose.position.x = 6.0F;
  pose.position.y = 3.0F;
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kCenter);
  assert(decision.rearm_required);

  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kCenter);

  referee.ammo = 50;
  referee.robot_hp = 180;
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(!decision.rearm_required);

  referee.robot_hp = 80;
  decision = controller.Update(pose.stamp, referee, pose, planner_status);
  assert(decision.phase == rm_nav::app::MatchModePhase::kReturnToSpawn);
  assert(decision.reason == "hp_low");

  return 0;
}
