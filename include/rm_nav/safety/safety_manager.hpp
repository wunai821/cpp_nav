#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/safety_event.hpp"
#include "rm_nav/safety/collision_checker.hpp"
#include "rm_nav/safety/command_gate.hpp"
#include "rm_nav/safety/deadman_checker.hpp"
#include "rm_nav/safety/failover_policy.hpp"
#include "rm_nav/safety/watchdog_heartbeat.hpp"

namespace rm_nav::safety {

enum class SafetyCommandAuthority {
  kAllow = 0,
  kLimited,
  kFreeze,
  kFailsafe,
};

const char* ToString(SafetyCommandAuthority authority);

struct SafetyInput {
  common::TimePoint stamp{};
  bool start_signal_active{false};
  bool arming_ready{false};
  bool navigation_requested{false};
  bool costmap_required{true};
  bool mode_transition_active{false};
  bool force_failsafe{false};
  bool planner_path_available{false};
  bool planner_global_plan_succeeded{false};
  bool planner_local_plan_succeeded{false};
  bool localization_degraded{false};
  bool localization_pose_trusted{false};
  float localization_match_score{0.0F};
  std::uint32_t localization_consecutive_bad_frames{0};
  bool planner_failed{false};
  bool goal_reached{false};
  bool mission_timeout_enabled{true};
  common::TimeNs last_communication_rx_ns{0};
  common::TimePoint last_chassis_feedback_stamp{};
  const data::Pose3f* current_pose{nullptr};
  const data::GridMap2D* costmap{nullptr};
  const std::vector<data::DynamicObstacle>* obstacles{nullptr};
  const data::ChassisCmd* proposed_cmd{nullptr};
};

struct SafetyResult {
  SafetyState state{SafetyState::kIdle};
  SafetyCommandAuthority authority{SafetyCommandAuthority::kFreeze};
  data::ChassisCmd allow_cmd{};
  data::ChassisCmd limited_cmd{};
  data::ChassisCmd freeze_cmd{};
  data::ChassisCmd failsafe_cmd{};
  data::ChassisCmd gated_cmd{};
  data::SafetyEvent event{};
  bool has_event{false};
  bool motion_allowed{false};
  bool planner_cmd_timed_out{false};
  bool communication_ok{true};
  bool chassis_feedback_ok{true};
  bool costmap_valid{false};
  bool costmap_fresh{false};
  bool localization_quality_ok{false};
  bool planner_status_ok{false};
  bool mode_transition_active{false};
  bool force_failsafe{false};
  bool gate_limited{false};
  CollisionType collision_type{CollisionType::kNone};
  bool obstacle_too_close{false};
  CommandGateReason gate_reason{CommandGateReason::kNone};
};

class SafetyManager {
 public:
  common::Status Configure(const config::SafetyConfig& config);
  common::Status Evaluate(const SafetyInput& input, SafetyResult* result);
  SafetyState state() const { return failover_policy_.state(); }

 private:
  config::SafetyConfig config_{};
  CollisionChecker collision_checker_{};
  CommandGate command_gate_{};
  DeadmanChecker deadman_checker_{};
  FailoverPolicy failover_policy_{};
  WatchdogHeartbeat watchdog_heartbeat_{};
  common::TimePoint last_mode_transition_at_{};
  bool configured_{false};
};

}  // namespace rm_nav::safety
