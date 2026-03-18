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

struct SafetyInput {
  common::TimePoint stamp{};
  bool start_signal_active{false};
  bool arming_ready{false};
  bool navigation_requested{false};
  bool planner_path_available{false};
  bool localization_degraded{false};
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
  data::ChassisCmd gated_cmd{};
  data::SafetyEvent event{};
  bool has_event{false};
  bool motion_allowed{false};
  bool planner_cmd_timed_out{false};
  bool communication_ok{true};
  bool chassis_feedback_ok{true};
  bool costmap_valid{false};
  CollisionType collision_type{CollisionType::kNone};
  bool obstacle_too_close{false};
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
  bool configured_{false};
};

}  // namespace rm_nav::safety
