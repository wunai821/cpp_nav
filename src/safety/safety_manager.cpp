#include "rm_nav/safety/safety_manager.hpp"

#include <chrono>
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

data::ChassisCmd BrakeCommand(common::TimePoint stamp) {
  data::ChassisCmd cmd;
  cmd.stamp = stamp;
  cmd.brake = true;
  return cmd;
}

CommandGateReason FailsafeReason(const SafetyResult& result) {
  if (!result.communication_ok) {
    return CommandGateReason::kCommunicationLost;
  }
  if (!result.chassis_feedback_ok) {
    return CommandGateReason::kChassisFeedbackLost;
  }
  if (!result.costmap_valid) {
    return CommandGateReason::kCostmapInvalid;
  }
  if (!result.costmap_fresh) {
    return CommandGateReason::kCostmapStale;
  }
  if (result.planner_cmd_timed_out) {
    return CommandGateReason::kPlannerTimeout;
  }
  if (!result.localization_quality_ok) {
    return CommandGateReason::kLocalizationDegraded;
  }
  if (!result.planner_status_ok) {
    return CommandGateReason::kPlannerFailed;
  }
  if (result.collision_type == CollisionType::kStatic) {
    return CommandGateReason::kStaticCollision;
  }
  if (result.collision_type == CollisionType::kDynamic) {
    return CommandGateReason::kDynamicCollision;
  }
  return CommandGateReason::kStateBlocked;
}

bool CostmapFresh(const data::GridMap2D* costmap, common::TimePoint stamp,
                  const config::SafetyConfig& config) {
  if (costmap == nullptr || costmap->stamp == common::TimePoint{} || stamp == common::TimePoint{}) {
    return false;
  }
  if (config.costmap_timeout_ms <= 0) {
    return true;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(stamp - costmap->stamp).count() <=
         config.costmap_timeout_ms;
}

}  // namespace

const char* ToString(SafetyCommandAuthority authority) {
  switch (authority) {
    case SafetyCommandAuthority::kAllow:
      return "allow_cmd";
    case SafetyCommandAuthority::kLimited:
      return "limited_cmd";
    case SafetyCommandAuthority::kFreeze:
      return "freeze_cmd";
    case SafetyCommandAuthority::kFailsafe:
      return "failsafe_cmd";
  }
  return "unknown";
}

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
  last_mode_transition_at_ = {};
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
  result->costmap_fresh = result->costmap_valid && CostmapFresh(input.costmap, input.stamp, config_);
  result->communication_ok = watchdog_heartbeat_.IsAlive(
      input.last_communication_rx_ns, common::ToNanoseconds(input.stamp), config_);
  result->chassis_feedback_ok =
      input.last_chassis_feedback_stamp == common::TimePoint{} ||
      watchdog_heartbeat_.IsAlive(input.last_chassis_feedback_stamp, input.stamp, config_);
  result->localization_quality_ok = input.localization_pose_trusted && !input.localization_degraded;
  result->planner_status_ok =
      input.planner_path_available && input.planner_global_plan_succeeded &&
      input.planner_local_plan_succeeded && !input.planner_failed;
  result->force_failsafe = input.force_failsafe;

  if (input.mode_transition_active) {
    last_mode_transition_at_ = input.stamp;
  }
  result->mode_transition_active =
      last_mode_transition_at_ != common::TimePoint{} &&
      std::chrono::duration_cast<std::chrono::milliseconds>(input.stamp - last_mode_transition_at_)
              .count() <= std::max(0, config_.mode_transition_freeze_ms);

  data::ChassisCmd proposed_cmd;
  if (input.proposed_cmd != nullptr) {
    proposed_cmd = *input.proposed_cmd;
  } else {
    proposed_cmd.brake = true;
  }
  proposed_cmd.stamp = input.stamp;
  result->allow_cmd = proposed_cmd;
  result->limited_cmd = proposed_cmd;
  result->freeze_cmd = BrakeCommand(input.stamp);
  result->failsafe_cmd = BrakeCommand(input.stamp);

  result->planner_cmd_timed_out =
      deadman_checker_.PlannerCmdTimedOut(proposed_cmd, input.stamp, config_);

  const data::Pose3f pose = input.current_pose != nullptr ? *input.current_pose : data::Pose3f{};
  const std::vector<data::DynamicObstacle> no_obstacles;
  const auto& obstacles = input.obstacles != nullptr ? *input.obstacles : no_obstacles;

  if (pose.is_valid && result->costmap_valid && !result->planner_cmd_timed_out &&
      result->costmap_fresh &&
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
  policy_input.costmap_required = input.costmap_required;
  policy_input.planner_path_available = input.planner_path_available;
  policy_input.planner_global_plan_succeeded = input.planner_global_plan_succeeded;
  policy_input.planner_local_plan_succeeded = input.planner_local_plan_succeeded;
  policy_input.planner_cmd_fresh = !result->planner_cmd_timed_out;
  policy_input.localization_degraded = input.localization_degraded;
  policy_input.localization_pose_trusted = input.localization_pose_trusted;
  policy_input.localization_match_score = input.localization_match_score;
  policy_input.planner_failed = input.planner_failed;
  policy_input.costmap_valid = result->costmap_valid;
  policy_input.costmap_fresh = result->costmap_fresh;
  policy_input.communication_ok = result->communication_ok;
  policy_input.chassis_feedback_ok = result->chassis_feedback_ok;
  policy_input.goal_reached = input.goal_reached;
  policy_input.mission_timeout_enabled = input.mission_timeout_enabled;
  policy_input.mode_transition_active = result->mode_transition_active;
  policy_input.force_failsafe = input.force_failsafe;
  policy_input.obstacle_too_close = result->obstacle_too_close;
  policy_input.collision_type = result->collision_type;

  const auto decision = failover_policy_.Evaluate(policy_input);
  result->state = decision.state;
  result->motion_allowed = decision.motion_allowed;
  result->event = decision.event;
  result->has_event = decision.has_event;

  if (input.force_failsafe || decision.state == SafetyState::kFailsafe) {
    result->authority = SafetyCommandAuthority::kFailsafe;
    result->gated_cmd = result->failsafe_cmd;
    result->gate_reason =
        input.force_failsafe ? CommandGateReason::kFailsafeOverride : FailsafeReason(*result);
    command_gate_.Reset();
    return common::Status::Ok();
  }

  if (result->mode_transition_active) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kModeTransition;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (!result->communication_ok) {
    result->authority = SafetyCommandAuthority::kFailsafe;
    result->gated_cmd = result->failsafe_cmd;
    result->gate_reason = CommandGateReason::kCommunicationLost;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (!result->chassis_feedback_ok) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kChassisFeedbackLost;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (input.costmap_required && !result->costmap_valid) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kCostmapInvalid;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (input.costmap_required && !result->costmap_fresh) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kCostmapStale;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (result->planner_cmd_timed_out) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kPlannerTimeout;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (!result->localization_quality_ok) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kLocalizationDegraded;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (!result->planner_status_ok) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kPlannerFailed;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (result->collision_type == CollisionType::kStatic) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kStaticCollision;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (result->collision_type == CollisionType::kDynamic) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = CommandGateReason::kDynamicCollision;
    command_gate_.Reset();
    return common::Status::Ok();
  }
  if (!decision.motion_allowed || proposed_cmd.brake || !HasMotionRequest(proposed_cmd)) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    result->gate_reason = proposed_cmd.brake ? CommandGateReason::kBrakeRequested
                                             : CommandGateReason::kStateBlocked;
    command_gate_.Reset();
    return common::Status::Ok();
  }

  const auto gate_result = command_gate_.Gate(proposed_cmd, input.stamp, true, CollisionType::kNone);
  result->limited_cmd = gate_result.command;
  result->gate_reason = gate_result.reason;
  result->gate_limited = gate_result.limited;
  if (gate_result.blocked) {
    result->authority = SafetyCommandAuthority::kFreeze;
    result->gated_cmd = result->freeze_cmd;
    if (result->gate_reason == CommandGateReason::kNone) {
      result->gate_reason = CommandGateReason::kStateBlocked;
    }
    return common::Status::Ok();
  }
  result->authority = gate_result.limited ? SafetyCommandAuthority::kLimited
                                          : SafetyCommandAuthority::kAllow;
  result->gated_cmd = gate_result.command;
  if (!gate_result.limited) {
    result->limited_cmd = gate_result.command;
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::safety
