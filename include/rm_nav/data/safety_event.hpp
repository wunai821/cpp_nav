#pragma once

#include <string_view>

#include "rm_nav/common/time.hpp"

namespace rm_nav::data {

enum class SafetySeverity {
  kInfo = 0,
  kWarning,
  kCritical,
};

enum class SafetyEventCode {
  kNone = 0,
  kSensorTimeout,
  kDeadmanTimeout,
  kPoseLost,
  kPlannerStall,
  kObstacleTooClose,
  kStaticCollision,
  kDynamicCollision,
  kEmergencyStop,
  kFailsafeOverride,
};

struct SafetyEvent {
  common::TimePoint stamp{};
  SafetyEventCode code{SafetyEventCode::kNone};
  SafetySeverity severity{SafetySeverity::kInfo};
  std::string_view message{};
};

}  // namespace rm_nav::data
