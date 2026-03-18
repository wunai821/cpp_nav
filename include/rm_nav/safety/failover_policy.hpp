#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/safety_config.hpp"
#include "rm_nav/data/safety_event.hpp"
#include "rm_nav/safety/collision_checker.hpp"

namespace rm_nav::safety {

enum class SafetyState {
  kIdle = 0,
  kArmed,
  kRunning,
  kHold,
  kFailsafe,
};

const char* ToString(SafetyState state);

struct SafetyPolicyInput {
  common::TimePoint stamp{};
  bool start_signal_active{false};
  bool arming_ready{false};
  bool navigation_requested{false};
  bool motion_requested{false};
  bool planner_path_available{false};
  bool planner_cmd_fresh{false};
  bool localization_degraded{false};
  bool planner_failed{false};
  bool costmap_valid{false};
  bool communication_ok{true};
  bool chassis_feedback_ok{true};
  bool goal_reached{false};
  bool mission_timeout_enabled{true};
  bool obstacle_too_close{false};
  CollisionType collision_type{CollisionType::kNone};
};

struct SafetyPolicyDecision {
  SafetyState state{SafetyState::kIdle};
  bool motion_allowed{false};
  data::SafetyEvent event{};
  bool has_event{false};
};

class FailoverPolicy {
 public:
  common::Status Configure(const config::SafetyConfig& config);
  SafetyPolicyDecision Evaluate(const SafetyPolicyInput& input);
  SafetyState state() const { return state_; }

 private:
  void UpdateLatchedTime(bool condition, common::TimePoint stamp,
                         common::TimePoint* since) const;
  bool TimedOut(common::TimePoint since, common::TimePoint now, int timeout_ms) const;
  data::SafetyEvent MakeEvent(common::TimePoint stamp, data::SafetyEventCode code,
                              data::SafetySeverity severity,
                              std::string_view message) const;
  void ResetMissionTimer();

  config::SafetyConfig config_{};
  SafetyState state_{SafetyState::kIdle};
  common::TimePoint hold_issue_since_{};
  common::TimePoint planner_issue_since_{};
  common::TimePoint localization_issue_since_{};
  common::TimePoint mission_started_at_{};
  bool configured_{false};
};

}  // namespace rm_nav::safety
