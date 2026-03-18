#include "rm_nav/safety/safety_manager.hpp"

#include <cmath>

namespace rm_nav::safety {
namespace {

bool HasMotionRequest(const data::ChassisCmd& cmd) {
  return !cmd.brake &&
         (std::fabs(cmd.vx_mps) + std::fabs(cmd.vy_mps) + std::fabs(cmd.wz_radps)) > 0.01F;
}

bool CostmapValid(const data::GridMap2D* costmap) {
  return costmap != nullptr && costmap->width > 0U && costmap->height > 0U &&
         costmap->resolution_m > 0.0F;
}

}  // namespace

common::Status SafetyManager::Configure(const config::SafetyConfig& config) {
  config_ = config;
  auto status = command_gate_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  status = failover_policy_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  configured_ = true;
  return common::Status::Ok();
}

common::Status SafetyManager::Evaluate(const SafetyInput& input, SafetyResult* result) {
  if (result == nullptr) {
    return common::Status::InvalidArgument("safety result output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("safety manager is not configured");
  }

  *result = {};
  result->costmap_valid = CostmapValid(input.costmap);
  result->communication_ok = watchdog_heartbeat_.IsAlive(
      input.last_communication_rx_ns, common::ToNanoseconds(input.stamp), config_);
  result->chassis_feedback_ok =
      input.last_chassis_feedback_stamp == common::TimePoint{} ||
      watchdog_heartbeat_.IsAlive(input.last_chassis_feedback_stamp, input.stamp, config_);

  data::ChassisCmd proposed_cmd;
  if (input.proposed_cmd != nullptr) {
    proposed_cmd = *input.proposed_cmd;
  } else {
    proposed_cmd.brake = true;
  }
  proposed_cmd.stamp = input.stamp;

  result->planner_cmd_timed_out =
      deadman_checker_.PlannerCmdTimedOut(proposed_cmd, input.stamp, config_);

  const data::Pose3f pose = input.current_pose != nullptr ? *input.current_pose : data::Pose3f{};
  const std::vector<data::DynamicObstacle> no_obstacles;
  const auto& obstacles = input.obstacles != nullptr ? *input.obstacles : no_obstacles;

  if (pose.is_valid && result->costmap_valid && !result->planner_cmd_timed_out &&
      HasMotionRequest(proposed_cmd)) {
    const auto collision_result = collision_checker_.Evaluate(
        pose, *input.costmap, obstacles, proposed_cmd, config_);
    result->collision_type = collision_result.type;
    result->obstacle_too_close = collision_result.obstacle_too_close;
  } else if (pose.is_valid) {
    result->obstacle_too_close =
        collision_checker_.Evaluate(pose, data::GridMap2D{}, obstacles, data::ChassisCmd{}, config_)
            .obstacle_too_close;
  }

  SafetyPolicyInput policy_input;
  policy_input.stamp = input.stamp;
  policy_input.start_signal_active = input.start_signal_active;
  policy_input.arming_ready = input.arming_ready;
  policy_input.navigation_requested = input.navigation_requested;
  policy_input.motion_requested = HasMotionRequest(proposed_cmd);
  policy_input.planner_path_available = input.planner_path_available;
  policy_input.planner_cmd_fresh = !result->planner_cmd_timed_out;
  policy_input.localization_degraded = input.localization_degraded;
  policy_input.planner_failed = input.planner_failed;
  policy_input.costmap_valid = result->costmap_valid;
  policy_input.communication_ok = result->communication_ok;
  policy_input.chassis_feedback_ok = result->chassis_feedback_ok;
  policy_input.goal_reached = input.goal_reached;
  policy_input.mission_timeout_enabled = input.mission_timeout_enabled;
  policy_input.obstacle_too_close = result->obstacle_too_close;
  policy_input.collision_type = result->collision_type;

  const auto decision = failover_policy_.Evaluate(policy_input);
  result->state = decision.state;
  result->motion_allowed = decision.motion_allowed;
  result->event = decision.event;
  result->has_event = decision.has_event;

  const auto gate_result = command_gate_.Gate(
      proposed_cmd, input.stamp, decision.motion_allowed, result->collision_type);
  result->gated_cmd = gate_result.command;
  return common::Status::Ok();
}

}  // namespace rm_nav::safety
