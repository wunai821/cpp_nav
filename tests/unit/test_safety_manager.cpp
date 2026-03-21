#include <cassert>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/safety/safety_manager.hpp"

namespace {

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.position.x = 1.0F;
  pose.position.y = 2.0F;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::GridMap2D MakeCostmap(rm_nav::common::TimePoint stamp, bool blocked) {
  rm_nav::data::GridMap2D costmap;
  costmap.stamp = stamp;
  costmap.width = 21U;
  costmap.height = 21U;
  costmap.resolution_m = 0.1F;
  costmap.occupancy.assign(costmap.width * costmap.height, 0U);
  if (blocked) {
    const std::size_t center_y = costmap.height / 2U;
    const std::size_t center_x = costmap.width / 2U;
    costmap.occupancy[(center_y + 3U) * costmap.width + (center_x + 3U)] = 100U;
  }
  return costmap;
}

rm_nav::data::ChassisCmd MakeMotionCmd(rm_nav::common::TimePoint stamp) {
  rm_nav::data::ChassisCmd cmd;
  cmd.stamp = stamp;
  cmd.vx_mps = 0.4F;
  cmd.brake = false;
  return cmd;
}

rm_nav::safety::SafetyInput MakeBaseInput(rm_nav::common::TimePoint stamp,
                                          const rm_nav::data::Pose3f* pose,
                                          const rm_nav::data::GridMap2D* costmap,
                                          const rm_nav::data::ChassisCmd* cmd,
                                          const std::vector<rm_nav::data::DynamicObstacle>* obstacles) {
  rm_nav::safety::SafetyInput input;
  input.stamp = stamp;
  input.start_signal_active = true;
  input.arming_ready = true;
  input.navigation_requested = true;
  input.planner_path_available = true;
  input.planner_global_plan_succeeded = true;
  input.planner_local_plan_succeeded = true;
  input.localization_pose_trusted = true;
  input.localization_match_score = 1.0F;
  input.last_communication_rx_ns = rm_nav::common::ToNanoseconds(stamp);
  input.last_chassis_feedback_stamp = stamp;
  input.current_pose = pose;
  input.costmap = costmap;
  input.obstacles = obstacles;
  input.proposed_cmd = cmd;
  return input;
}

}  // namespace

int main() {
  rm_nav::safety::SafetyManager manager;
  rm_nav::config::SafetyConfig config;
  config.heartbeat_timeout_ms = 100;
  config.deadman_timeout_ms = 100;
  config.costmap_timeout_ms = 120;
  config.mode_transition_freeze_ms = 100;
  config.hold_timeout_ms = 250;
  config.planner_fail_timeout_ms = 250;
  config.localization_fail_timeout_ms = 150;
  config.mission_timeout_ms = 1000;
  config.max_vx_mps = 0.5;
  config.max_vy_mps = 0.4;
  config.max_wz_radps = 0.6;
  config.max_delta_v_per_tick = 0.1;
  config.max_delta_w_per_tick = 0.2;
  assert(manager.Configure(config).ok());

  const auto base_time = rm_nav::common::FromNanoseconds(1000000000LL);
  const std::vector<rm_nav::data::DynamicObstacle> no_obstacles;

  auto pose = MakePose(base_time);
  auto costmap = MakeCostmap(base_time, false);
  auto cmd = MakeMotionCmd(base_time);

  {
    auto input = MakeBaseInput(base_time, &pose, &costmap, &cmd, &no_obstacles);
    input.start_signal_active = false;
    input.arming_ready = false;
    input.navigation_requested = false;
    input.proposed_cmd = nullptr;

    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kIdle);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gated_cmd.brake);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(50);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    rm_nav::data::ChassisCmd brake_cmd;
    brake_cmd.stamp = stamp;
    brake_cmd.brake = true;

    auto input = MakeBaseInput(stamp, &pose, &costmap, &brake_cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kArmed);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gated_cmd.brake);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(100);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kRunning);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kLimited);
    assert(!result.gated_cmd.brake);
    assert(result.gated_cmd.vx_mps > 0.0F);
    assert(result.gated_cmd.vx_mps <= static_cast<float>(config.max_delta_v_per_tick) + 1.0e-5F);
    assert(result.allow_cmd.vx_mps == cmd.vx_mps);
    assert(result.limited_cmd.vx_mps == result.gated_cmd.vx_mps);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(150);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.localization_degraded = true;
    input.localization_pose_trusted = false;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kHold);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gated_cmd.brake);
    assert(result.has_event);
    assert(result.event.code == rm_nav::data::SafetyEventCode::kPoseLost);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(350);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.localization_degraded = true;
    input.localization_pose_trusted = false;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kFailsafe);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFailsafe);
    assert(result.gated_cmd.brake);
    assert(result.has_event);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(400);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);

    auto input = MakeBaseInput(stamp, &pose, &costmap, nullptr, &no_obstacles);
    input.start_signal_active = false;
    input.navigation_requested = false;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kIdle);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(450);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    rm_nav::data::ChassisCmd brake_cmd;
    brake_cmd.stamp = stamp;
    brake_cmd.brake = true;

    auto input = MakeBaseInput(stamp, &pose, &costmap, &brake_cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kArmed);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(500);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kRunning);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kLimited ||
           result.authority == rm_nav::safety::SafetyCommandAuthority::kAllow);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(550);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, true);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kHold);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gated_cmd.brake);
    assert(result.event.code == rm_nav::data::SafetyEventCode::kStaticCollision);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(600);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kRunning);
    assert(!result.gated_cmd.brake);
    assert(result.gated_cmd.vx_mps <= static_cast<float>(config.max_vx_mps) + 1.0e-5F);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(750);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.last_communication_rx_ns = rm_nav::common::ToNanoseconds(base_time);
    input.last_chassis_feedback_stamp = base_time;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kFailsafe);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFailsafe);
    assert(result.gated_cmd.brake);
    assert(result.has_event);
    assert(result.event.code == rm_nav::data::SafetyEventCode::kSensorTimeout);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(800);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.mode_transition_active = true;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kHold);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gate_reason == rm_nav::safety::CommandGateReason::kModeTransition);
  }

  assert(manager.Configure(config).ok());

  {
    const auto stamp = base_time + std::chrono::milliseconds(820);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.mode_transition_active = true;
    input.last_communication_rx_ns = rm_nav::common::ToNanoseconds(base_time);
    input.last_chassis_feedback_stamp = base_time;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kFailsafe);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFailsafe);
    assert(result.gate_reason == rm_nav::safety::CommandGateReason::kCommunicationLost);
  }

  assert(manager.Configure(config).ok());

  {
    const auto stamp = base_time + std::chrono::milliseconds(840);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.mode_transition_active = true;
    input.localization_degraded = true;
    input.localization_pose_trusted = false;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kHold);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gate_reason == rm_nav::safety::CommandGateReason::kModeTransition);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(950);
    pose = MakePose(stamp);
    costmap = MakeCostmap(base_time + std::chrono::milliseconds(700), false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(!result.costmap_fresh);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFreeze);
    assert(result.gate_reason == rm_nav::safety::CommandGateReason::kCostmapStale);
  }

  {
    const auto stamp = base_time + std::chrono::milliseconds(1000);
    pose = MakePose(stamp);
    costmap = MakeCostmap(stamp, false);
    cmd = MakeMotionCmd(stamp);

    auto input = MakeBaseInput(stamp, &pose, &costmap, &cmd, &no_obstacles);
    input.force_failsafe = true;
    rm_nav::safety::SafetyResult result;
    assert(manager.Evaluate(input, &result).ok());
    assert(result.state == rm_nav::safety::SafetyState::kFailsafe);
    assert(result.authority == rm_nav::safety::SafetyCommandAuthority::kFailsafe);
    assert(result.gate_reason == rm_nav::safety::CommandGateReason::kFailsafeOverride);
  }

  std::cout << "test_safety_manager passed\n";
  return 0;
}
