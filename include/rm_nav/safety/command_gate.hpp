#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/safety_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/safety/collision_checker.hpp"

namespace rm_nav::safety {

enum class CommandGateReason {
  kNone = 0,
  kStateBlocked,
  kBrakeRequested,
  kStaticCollision,
  kDynamicCollision,
};

struct CommandGateResult {
  data::ChassisCmd command{};
  bool blocked{false};
  CommandGateReason reason{CommandGateReason::kNone};
};

class CommandGate {
 public:
  common::Status Configure(const config::SafetyConfig& config);
  void Reset();
  CommandGateResult Gate(const data::ChassisCmd& proposed_cmd, common::TimePoint stamp,
                         bool motion_allowed, CollisionType collision_type);

 private:
  config::SafetyConfig config_{};
  data::ChassisCmd last_command_{};
  bool configured_{false};
};

}  // namespace rm_nav::safety
