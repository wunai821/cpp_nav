#include "rm_nav/safety/failover_policy.hpp"

#include <algorithm>

namespace rm_nav::safety {

const char* ToString(SafetyState state) {
  switch (state) {
    case SafetyState::kIdle:
      return "IDLE";
    case SafetyState::kArmed:
      return "ARMED";
    case SafetyState::kRunning:
      return "RUNNING";
    case SafetyState::kHold:
      return "HOLD";
    case SafetyState::kFailsafe:
      return "FAILSAFE";
  }
  return "UNKNOWN";
}

common::Status FailoverPolicy::Configure(const config::SafetyConfig& config) {
  config_ = config;
  state_ = SafetyState::kIdle;
  hold_issue_since_ = {};
  planner_issue_since_ = {};
  localization_issue_since_ = {};
  mission_started_at_ = {};
  configured_ = true;
  return common::Status::Ok();
}

void FailoverPolicy::UpdateLatchedTime(bool condition, common::TimePoint stamp,
                                       common::TimePoint* since) const {
  if (since == nullptr) {
    return;
  }
  if (!condition) {
    *since = {};
    return;
  }
  if (*since == common::TimePoint{}) {
    *since = stamp;
  }
}

bool FailoverPolicy::TimedOut(common::TimePoint since, common::TimePoint now, int timeout_ms) const {
  if (since == common::TimePoint{} || timeout_ms <= 0) {
    return false;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - since).count() >= timeout_ms;
}

data::SafetyEvent FailoverPolicy::MakeEvent(common::TimePoint stamp, data::SafetyEventCode code,
                                            data::SafetySeverity severity,
                                            std::string_view message) const {
  data::SafetyEvent event;
  event.stamp = stamp;
  event.code = code;
  event.severity = severity;
  event.message = message;
  return event;
}

void FailoverPolicy::ResetMissionTimer() { mission_started_at_ = {}; }

SafetyPolicyDecision FailoverPolicy::Evaluate(const SafetyPolicyInput& input) {
  SafetyPolicyDecision decision;
  decision.state = state_;

  if (!configured_) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kFailsafeOverride,
                  data::SafetySeverity::kCritical, "safety policy is not configured");
    decision.state = SafetyState::kFailsafe;
    state_ = decision.state;
    return decision;
  }

  const bool planner_issue =
      input.planner_failed || !input.planner_path_available || !input.planner_cmd_fresh ||
      !input.planner_global_plan_succeeded || !input.planner_local_plan_succeeded;
  const bool localization_issue =
      input.localization_degraded || !input.localization_pose_trusted;
  const bool costmap_issue = input.costmap_required && (!input.costmap_valid || !input.costmap_fresh);
  const bool transient_hold =
      input.collision_type != CollisionType::kNone || planner_issue || localization_issue ||
      costmap_issue || !input.chassis_feedback_ok || input.mode_transition_active;

  UpdateLatchedTime(transient_hold, input.stamp, &hold_issue_since_);
  UpdateLatchedTime(planner_issue, input.stamp, &planner_issue_since_);
  UpdateLatchedTime(localization_issue, input.stamp, &localization_issue_since_);

  if (input.mission_timeout_enabled && input.navigation_requested && !input.goal_reached) {
    if (mission_started_at_ == common::TimePoint{}) {
      mission_started_at_ = input.stamp;
    }
  } else {
    ResetMissionTimer();
  }

  const bool hold_timeout = TimedOut(hold_issue_since_, input.stamp, config_.hold_timeout_ms);
  const bool planner_timeout =
      TimedOut(planner_issue_since_, input.stamp, config_.planner_fail_timeout_ms);
  const bool localization_timeout =
      TimedOut(localization_issue_since_, input.stamp, config_.localization_fail_timeout_ms);
  const bool mission_timeout =
      input.mission_timeout_enabled &&
      TimedOut(mission_started_at_, input.stamp, config_.mission_timeout_ms);

  const bool fatal = input.force_failsafe || !input.communication_ok || input.obstacle_too_close ||
                     localization_timeout || planner_timeout || hold_timeout || mission_timeout;

  if (fatal) {
    state_ = SafetyState::kFailsafe;
  } else {
    switch (state_) {
      case SafetyState::kIdle:
        if (input.start_signal_active && input.arming_ready && input.localization_pose_trusted &&
            !input.localization_degraded && (!input.costmap_required ||
                                             (input.costmap_valid && input.costmap_fresh)) &&
            !input.mode_transition_active) {
          state_ = SafetyState::kArmed;
        }
        break;
      case SafetyState::kArmed:
        if (!input.start_signal_active || !input.arming_ready) {
          state_ = SafetyState::kIdle;
        } else if (input.navigation_requested && input.motion_requested && !transient_hold) {
          state_ = SafetyState::kRunning;
        }
        break;
      case SafetyState::kRunning:
        if (!input.start_signal_active || !input.arming_ready) {
          state_ = SafetyState::kIdle;
        } else if (transient_hold || !input.motion_requested) {
          state_ = SafetyState::kHold;
        }
        break;
      case SafetyState::kHold:
        if (!input.start_signal_active || !input.arming_ready) {
          state_ = SafetyState::kIdle;
        } else if (!transient_hold) {
          state_ = input.navigation_requested && input.motion_requested ? SafetyState::kRunning
                                                                        : SafetyState::kArmed;
        }
        break;
      case SafetyState::kFailsafe:
        if (!input.start_signal_active && !input.navigation_requested) {
          state_ = SafetyState::kIdle;
          hold_issue_since_ = {};
          planner_issue_since_ = {};
          localization_issue_since_ = {};
          ResetMissionTimer();
        }
        break;
    }
  }

  decision.state = state_;
  decision.motion_allowed = state_ == SafetyState::kRunning;

  if (!input.communication_ok) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kSensorTimeout,
                  data::SafetySeverity::kCritical, "communication watchdog timeout");
    return decision;
  }
  if (input.obstacle_too_close) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kEmergencyStop,
                  data::SafetySeverity::kCritical, "obstacle inside emergency stop distance");
    return decision;
  }
  if (mission_timeout) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kFailsafeOverride,
                  data::SafetySeverity::kCritical, "goal timeout exceeded");
    return decision;
  }
  if (localization_timeout) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kPoseLost,
                  data::SafetySeverity::kCritical, "localization loss escalated to failsafe");
    return decision;
  }
  if (planner_timeout || hold_timeout) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kFailsafeOverride,
                  data::SafetySeverity::kCritical, "hold timeout escalated to failsafe");
    return decision;
  }
  if (input.collision_type == CollisionType::kStatic) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kStaticCollision,
                  data::SafetySeverity::kCritical, "command blocked by static collision");
    return decision;
  }
  if (input.collision_type == CollisionType::kDynamic) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kDynamicCollision,
                  data::SafetySeverity::kCritical, "command blocked by dynamic collision");
    return decision;
  }
  if (!input.planner_cmd_fresh) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kDeadmanTimeout,
                  data::SafetySeverity::kWarning, "planner command deadman timeout");
    return decision;
  }
  if (input.localization_degraded) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kPoseLost,
                  data::SafetySeverity::kWarning, "localization degraded");
    return decision;
  }
  if (input.planner_failed || !input.planner_path_available) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kPlannerStall,
                  data::SafetySeverity::kWarning, "planner path unavailable");
    return decision;
  }
  if ((input.costmap_required && (!input.costmap_valid || !input.costmap_fresh)) ||
      !input.chassis_feedback_ok) {
    decision.has_event = true;
    decision.event =
        MakeEvent(input.stamp, data::SafetyEventCode::kSensorTimeout,
                  data::SafetySeverity::kWarning, "costmap or chassis feedback unavailable");
    return decision;
  }

  return decision;
}

}  // namespace rm_nav::safety
