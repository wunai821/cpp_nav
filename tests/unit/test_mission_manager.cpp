#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/planning/mission_manager.hpp"

namespace {

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.is_valid = true;
  return pose;
}

rm_nav::planning::GoalState MakeGoal(float distance_m, float yaw_error_rad) {
  rm_nav::planning::GoalState goal;
  goal.target_pose.is_valid = true;
  goal.distance_to_target_m = distance_m;
  goal.distance_to_center_m = distance_m;
  goal.yaw_error_rad = yaw_error_rad;
  goal.within_center_radius = distance_m <= 0.6F;
  goal.yaw_aligned = std::fabs(yaw_error_rad) <= 0.12F;
  return goal;
}

}  // namespace

int main() {
  rm_nav::config::PlannerConfig config;
  config.center_radius_m = 0.6;
  config.yaw_align_tolerance_rad = 0.12;
  config.center_hold_settle_frames = 3;
  config.center_hold_settle_time_ms = 150;
  config.recenter_threshold_m = 0.9;

  rm_nav::planning::MissionManager manager(config);
  const auto start = rm_nav::common::Now();

  auto status = manager.Update(MakePose(start), MakeGoal(0.25F, 0.02F));
  assert(status.phase == rm_nav::planning::MissionPhase::kSettling);
  assert(status.settling);
  assert(!status.reached);
  assert(status.consecutive_in_goal_frames == 1);

  status = manager.Update(MakePose(start + std::chrono::milliseconds(70)),
                          MakeGoal(0.22F, 0.01F));
  assert(status.phase == rm_nav::planning::MissionPhase::kSettling);
  assert(status.consecutive_in_goal_frames == 2);
  assert(!status.reached);

  status = manager.Update(MakePose(start + std::chrono::milliseconds(180)),
                          MakeGoal(0.20F, 0.01F));
  assert(status.phase == rm_nav::planning::MissionPhase::kCenterHold);
  assert(status.mode == rm_nav::planning::GoalMode::kCenterHold);
  assert(status.reached);
  assert(status.consecutive_in_goal_frames == 3);
  assert(status.settle_elapsed_ns > 0);

  status = manager.Update(MakePose(start + std::chrono::milliseconds(260)),
                          MakeGoal(0.75F, 0.02F));
  assert(status.phase == rm_nav::planning::MissionPhase::kCenterHold);
  assert(status.reached);
  assert(!status.hold_drifted);

  status = manager.Update(MakePose(start + std::chrono::milliseconds(320)),
                          MakeGoal(1.05F, 0.02F));
  assert(status.phase == rm_nav::planning::MissionPhase::kApproach);
  assert(status.mode == rm_nav::planning::GoalMode::kApproachCenter);
  assert(!status.reached);
  assert(status.hold_drifted);

  std::cout << "test_mission_manager passed\n";
  return 0;
}
