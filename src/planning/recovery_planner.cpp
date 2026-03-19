#include "rm_nav/planning/recovery_planner.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::planning {
namespace {

data::ChassisCmd BrakeCommand(common::TimePoint stamp) {
  data::ChassisCmd cmd;
  cmd.stamp = stamp;
  cmd.brake = true;
  return cmd;
}

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

float ClampAbs(float value, float limit) {
  return std::max(-limit, std::min(limit, value));
}

bool ValidInput(const RecoveryPlannerInput& input) {
  return input.current_pose != nullptr && input.planner_status != nullptr &&
         input.localization_result != nullptr && input.nominal_cmd != nullptr;
}

}  // namespace

common::Status RecoveryPlanner::Configure(const config::PlannerConfig& config) {
  config_ = config;
  Reset();
  configured_ = true;
  return common::Status::Ok();
}

void RecoveryPlanner::Reset() {
  latest_status_ = {};
  active_tier_ = fsm::RecoveryTier::kNone;
  active_cause_ = fsm::RecoveryCause::kNone;
  failure_streak_ = 0;
  cycles_in_tier_ = 0;
  cooldown_ticks_remaining_ = 0;
}

fsm::RecoveryCause RecoveryPlanner::DetermineCause(const RecoveryPlannerInput& input) const {
  const auto& localization = *input.localization_result;
  const auto& planner_status = *input.planner_status;
  if (!localization.status.pose_trusted) {
    if (localization.status.rejection_reason == "map_to_odom_guard" ||
        localization.status.rejection_reason == "relocalization_stabilization_failed") {
      return fsm::RecoveryCause::kMapToOdomRejected;
    }
    return fsm::RecoveryCause::kLocalizationLost;
  }

  if (planner_status.failure_reason == "local_plan_failed" ||
      planner_status.failure_reason == "global_plan_failed") {
    const bool near_goal =
        planner_status.hold_drifted ||
        planner_status.distance_to_goal_m <
            static_cast<float>(std::max(0.8, config_.center_radius_m * 1.5));
    if (near_goal) {
      return fsm::RecoveryCause::kStuckNearGoal;
    }
    if (planner_status.dwa_score.dynamic_max_risk > 0.8F ||
        planner_status.dwa_score.dynamic_crossing_penalty > 0.2F) {
      return fsm::RecoveryCause::kDynamicBlock;
    }
    if (planner_status.dwa_score.clearance_score < 1.0F) {
      return fsm::RecoveryCause::kCostmapCongested;
    }
    return fsm::RecoveryCause::kLocalPlannerStall;
  }

  if (planner_status.fallback_cmd_used) {
    if (planner_status.dwa_score.dynamic_max_risk > 0.8F) {
      return fsm::RecoveryCause::kDynamicBlock;
    }
    return planner_status.distance_to_goal_m <
                   static_cast<float>(std::max(0.8, config_.center_radius_m * 1.5))
               ? fsm::RecoveryCause::kStuckNearGoal
               : fsm::RecoveryCause::kCostmapCongested;
  }
  return fsm::RecoveryCause::kNone;
}

void RecoveryPlanner::UpdateTierState(fsm::RecoveryCause cause) {
  if (cause == fsm::RecoveryCause::kNone) {
    return;
  }
  const auto minimum_tier = fsm::MinimumTierForCause(cause);
  if (active_cause_ != cause) {
    active_cause_ = cause;
    active_tier_ = minimum_tier;
    cycles_in_tier_ = 0;
    return;
  }

  active_tier_ = std::max(active_tier_, minimum_tier);
  ++cycles_in_tier_;
  if (active_tier_ == fsm::RecoveryTier::kLight &&
      cycles_in_tier_ > std::max(1, config_.recovery_light_escalate_cycles)) {
    active_tier_ = fsm::RecoveryTier::kMedium;
    cycles_in_tier_ = 0;
  } else if (active_tier_ == fsm::RecoveryTier::kMedium &&
             cycles_in_tier_ > std::max(1, config_.recovery_medium_escalate_cycles)) {
    active_tier_ = fsm::RecoveryTier::kHeavy;
    cycles_in_tier_ = 0;
  }
}

float RecoveryPlanner::PreferredLateralDirection(const RecoveryPlannerInput& input) const {
  if (input.obstacles == nullptr || input.obstacles->empty() || input.current_pose == nullptr) {
    return (cycles_in_tier_ / 6) % 2 == 0 ? 1.0F : -1.0F;
  }

  const auto& pose = *input.current_pose;
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  float weighted_side = 0.0F;
  for (const auto& obstacle : *input.obstacles) {
    if (!obstacle.pose.is_valid) {
      continue;
    }
    const float dx = obstacle.pose.position.x - pose.position.x;
    const float dy = obstacle.pose.position.y - pose.position.y;
    const float local_y = -sin_yaw * dx + cos_yaw * dy;
    weighted_side += local_y >= 0.0F ? -1.0F : 1.0F;
  }
  if (std::fabs(weighted_side) < 1.0e-4F) {
    return (cycles_in_tier_ / 6) % 2 == 0 ? 1.0F : -1.0F;
  }
  return weighted_side > 0.0F ? 1.0F : -1.0F;
}

data::Pose3f RecoveryPlanner::BuildReapproachGoal(const RecoveryPlannerInput& input) const {
  data::Pose3f goal;
  if (input.current_pose == nullptr || input.global_path == nullptr ||
      input.global_path->points.empty()) {
    return goal;
  }

  const auto& pose = *input.current_pose;
  const auto& path = *input.global_path;
  const std::size_t lookahead_index = std::min<std::size_t>(
      std::max(1, config_.recovery_reapproach_lookahead_points), path.points.size() - 1U);
  const auto& path_point = path.points[lookahead_index];
  const float lateral_sign = PreferredLateralDirection(input);
  const float heading = path_point.heading_rad;
  const float lateral_offset =
      static_cast<float>(std::max(0.0, config_.recovery_reapproach_lateral_offset_m));

  goal = *input.current_pose;
  goal.position.x = path_point.position.x - std::sin(heading) * lateral_offset * lateral_sign;
  goal.position.y = path_point.position.y + std::cos(heading) * lateral_offset * lateral_sign;
  goal.rpy.z = heading;
  goal.is_valid = true;
  goal.reference_frame = pose.reference_frame;
  goal.child_frame = pose.child_frame;
  goal.stamp = input.stamp;
  return goal;
}

data::ChassisCmd RecoveryPlanner::BuildLightRecoveryCommand(const RecoveryPlannerInput& input,
                                                            RecoveryPlannerStatus* status) {
  auto cmd = BrakeCommand(input.stamp);
  cmd.brake = false;
  const float lateral_sign = PreferredLateralDirection(input);
  const int phase = cycles_in_tier_ % 12;
  const float light_rotate =
      static_cast<float>(std::max(0.0, config_.recovery_light_rotate_wz_radps));
  const float light_shift =
      static_cast<float>(std::max(0.0, config_.recovery_light_shift_vy_mps));
  const float light_forward =
      static_cast<float>(std::max(0.0, config_.recovery_light_forward_vx_mps));
  status->clearance_weight_scale =
      static_cast<float>(std::max(1.0, config_.recovery_clearance_weight_scale));
  if (active_cause_ == fsm::RecoveryCause::kDynamicBlock) {
    if (phase < 4) {
      status->action = fsm::RecoveryAction::kSlowdownResample;
      status->detail = "yield_to_dynamic_gap";
      cmd.vx_mps =
          std::min(light_forward, std::max(0.0F, input.nominal_cmd->vx_mps * 0.35F));
      cmd.wz_radps = 0.5F * light_rotate * lateral_sign;
    } else if (phase < 8) {
      status->action = fsm::RecoveryAction::kRotateInPlace;
      status->detail = "scan_for_gap";
      cmd.wz_radps = light_rotate * lateral_sign;
    } else {
      status->action = fsm::RecoveryAction::kMicroSidestep;
      status->detail = "nudge_side_for_gap";
      cmd.vy_mps = light_shift * lateral_sign;
      cmd.wz_radps = 0.6F * light_rotate * lateral_sign;
    }
    status->strategy = "l1_dynamic_gap_release";
    return cmd;
  }

  if (phase < 4) {
    status->action = fsm::RecoveryAction::kMicroSidestep;
    status->detail = "clear_local_costmap_margin";
    cmd.vy_mps = light_shift * lateral_sign;
  } else if (phase < 8) {
    status->action = fsm::RecoveryAction::kRotateInPlace;
    status->detail = "resample_heading";
    cmd.wz_radps = light_rotate * lateral_sign;
  } else {
    status->action = fsm::RecoveryAction::kSlowdownResample;
    status->detail = "resample_local_target";
    cmd.vx_mps = light_forward;
    cmd.vy_mps = 0.6F * light_shift * lateral_sign;
  }
  status->strategy = "l1_costmap_release";
  return cmd;
}

data::ChassisCmd RecoveryPlanner::BuildMediumRecoveryCommand(const RecoveryPlannerInput& input,
                                                             RecoveryPlannerStatus* status) {
  auto cmd = BrakeCommand(input.stamp);
  cmd.brake = false;
  const float lateral_sign = PreferredLateralDirection(input);
  const int phase = cycles_in_tier_ % 18;
  const float medium_backoff =
      static_cast<float>(std::max(0.0, config_.recovery_medium_backoff_vx_mps));
  const float medium_shift =
      static_cast<float>(std::max(0.0, config_.recovery_medium_shift_vy_mps));
  const float medium_rotate =
      static_cast<float>(std::max(0.0, config_.recovery_medium_rotate_wz_radps));
  const float reapproach_forward =
      static_cast<float>(std::max(0.0, config_.recovery_reapproach_forward_vx_mps));
  status->clearance_weight_scale =
      static_cast<float>(std::max(1.0, config_.recovery_clearance_weight_scale));
  if (phase < 6) {
    status->action = fsm::RecoveryAction::kBackoff;
    status->detail = active_cause_ == fsm::RecoveryCause::kStuckNearGoal
                         ? "goal_area_backoff"
                         : "backoff_from_deadlock";
    cmd.vx_mps = -medium_backoff;
  } else if (phase < 12) {
    status->action = fsm::RecoveryAction::kEscapeSidestep;
    status->detail = "sidestep_escape";
    cmd.vy_mps = medium_shift * lateral_sign;
    cmd.wz_radps = medium_rotate * lateral_sign;
  } else {
    status->action = fsm::RecoveryAction::kReapproachWaypoint;
    status->detail = active_cause_ == fsm::RecoveryCause::kStuckNearGoal
                         ? "reselect_goal_approach_waypoint"
                         : "reapproach_after_escape";
    status->temporary_goal = BuildReapproachGoal(input);
    status->temporary_goal_valid = status->temporary_goal.is_valid;
    if (status->temporary_goal_valid && input.current_pose != nullptr) {
      const auto& pose = *input.current_pose;
      const float dx = status->temporary_goal.position.x - pose.position.x;
      const float dy = status->temporary_goal.position.y - pose.position.y;
      const float cos_yaw = std::cos(pose.rpy.z);
      const float sin_yaw = std::sin(pose.rpy.z);
      const float dx_body = cos_yaw * dx + sin_yaw * dy;
      const float dy_body = -sin_yaw * dx + cos_yaw * dy;
      cmd.vx_mps = ClampAbs(dx_body * 0.7F, reapproach_forward);
      cmd.vy_mps = ClampAbs(dy_body * 0.7F, medium_shift);
      cmd.wz_radps = ClampAbs(NormalizeAngle(status->temporary_goal.rpy.z - pose.rpy.z) * 0.8F,
                              medium_rotate);
    } else {
      cmd.vx_mps = reapproach_forward;
      cmd.vy_mps = -0.8F * medium_shift * lateral_sign;
      cmd.wz_radps = -1.1F * medium_rotate * lateral_sign;
    }
  }
  status->strategy = active_cause_ == fsm::RecoveryCause::kStuckNearGoal
                         ? "l2_goal_unstick"
                         : "l2_deadlock_escape";
  return cmd;
}

data::ChassisCmd RecoveryPlanner::BuildHeavyRecoveryCommand(const RecoveryPlannerInput& input,
                                                            RecoveryPlannerStatus* status) {
  if (!input.localization_result->status.pose_trusted ||
      input.localization_result->relocalization.active) {
    status->requires_relocalization = true;
  }

  if (cooldown_ticks_remaining_ > 0 && input.nominal_cmd != nullptr) {
    auto cmd = *input.nominal_cmd;
    cmd.stamp = input.stamp;
    cmd.brake = false;
    cmd.vx_mps *= static_cast<float>(std::max(0.0, config_.recovery_slow_restart_linear_scale));
    cmd.vy_mps *= static_cast<float>(std::max(0.0, config_.recovery_slow_restart_linear_scale));
    cmd.wz_radps *= static_cast<float>(std::max(0.0, config_.recovery_slow_restart_yaw_scale));
    --cooldown_ticks_remaining_;
    status->cooldown_active = true;
    status->action = fsm::RecoveryAction::kSlowRestart;
    status->strategy = "l3_slow_restart";
    status->detail = "post_relocalization_ramp";
    status->complete = cooldown_ticks_remaining_ == 0;
    return cmd;
  }

  status->action = fsm::RecoveryAction::kStopAndRelocalize;
  status->strategy = "l3_stop_and_relocalize";
  status->detail = active_cause_ == fsm::RecoveryCause::kMapToOdomRejected
                       ? "persistent_map_to_odom_rejection"
                       : "pose_untrusted_or_recovery_exhausted";
  if (cycles_in_tier_ > std::max(1, config_.recovery_heavy_exhaust_cycles)) {
    status->exhausted = true;
    status->cause = fsm::RecoveryCause::kRecoveryExhausted;
  }
  return BrakeCommand(input.stamp);
}

common::Status RecoveryPlanner::Plan(const RecoveryPlannerInput& input, data::ChassisCmd* cmd,
                                     RecoveryPlannerStatus* status) {
  if (cmd == nullptr || status == nullptr) {
    return common::Status::InvalidArgument("recovery planner output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("recovery planner is not configured");
  }
  if (!ValidInput(input)) {
    return common::Status::InvalidArgument("recovery planner input is incomplete");
  }

  RecoveryPlannerStatus next_status;
  next_status.active = true;
  next_status.failure_streak = failure_streak_;

  const auto cause = DetermineCause(input);
  if (cause == fsm::RecoveryCause::kNone) {
    if (active_tier_ == fsm::RecoveryTier::kHeavy && cooldown_ticks_remaining_ == 0) {
      cooldown_ticks_remaining_ = std::max(0, config_.recovery_heavy_restart_ticks);
    }
    if (cooldown_ticks_remaining_ > 0) {
      next_status.active = true;
      next_status.tier = fsm::RecoveryTier::kHeavy;
      next_status.cause = active_cause_;
      next_status.failure_streak = failure_streak_;
      next_status.cycles_in_tier = cycles_in_tier_;
      *cmd = BuildHeavyRecoveryCommand(input, &next_status);
      latest_status_ = next_status;
      *status = latest_status_;
      if (next_status.complete) {
        Reset();
      }
      return common::Status::Ok();
    }

    next_status.complete = true;
    next_status.active = false;
    next_status.detail = "recovered_nominal_navigation";
    *cmd = input.nominal_cmd != nullptr ? *input.nominal_cmd : BrakeCommand(input.stamp);
    latest_status_ = next_status;
    *status = latest_status_;
    Reset();
    return common::Status::Ok();
  }

  ++failure_streak_;
  UpdateTierState(cause);
  next_status.failure_streak = failure_streak_;
  next_status.tier = active_tier_;
  next_status.cause = active_cause_;
  next_status.cycles_in_tier = cycles_in_tier_;
  next_status.requires_relocalization = fsm::CauseNeedsRelocalization(active_cause_);

  switch (active_tier_) {
    case fsm::RecoveryTier::kLight:
      *cmd = BuildLightRecoveryCommand(input, &next_status);
      break;
    case fsm::RecoveryTier::kMedium:
      *cmd = BuildMediumRecoveryCommand(input, &next_status);
      break;
    case fsm::RecoveryTier::kHeavy:
      *cmd = BuildHeavyRecoveryCommand(input, &next_status);
      break;
    case fsm::RecoveryTier::kNone:
    default:
      *cmd = BrakeCommand(input.stamp);
      break;
  }

  cmd->stamp = input.stamp;
  latest_status_ = next_status;
  *status = latest_status_;
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
