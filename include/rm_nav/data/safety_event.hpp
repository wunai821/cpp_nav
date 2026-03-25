#pragma once

#include <string>

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
  std::string message{};
  std::string trigger_source{"none"};
  double trigger_threshold{0.0};
  std::uint32_t consecutive_bad_frames{0};
};

}  // namespace rm_nav::data
