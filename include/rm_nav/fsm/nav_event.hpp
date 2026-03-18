#pragma once

#include <string>

#include "rm_nav/common/time.hpp"

namespace rm_nav::fsm {

enum class NavEventCode {
  kNone = 0,
  kSelfCheckPassed,
  kEnterWarmupMode,
  kEnterCombatMode,
  kMapLoadSuccess,
  kMapSaveSuccess,
  kMapSaveWriteFailed,
  kMapValidationFailed,
  kMapStorageSwitchFailed,
  kMapUnavailable,
  kLocalizationDegraded,
  kPlannerFailed,
  kCenterDrifted,
  kSafetyTriggered,
  kHeartbeatAnomaly,
  kRefereeStateChanged,
};

struct NavEvent {
  common::TimePoint stamp{};
  NavEventCode code{NavEventCode::kNone};
  std::string summary{};
};

const char* ToString(NavEventCode code);

}  // namespace rm_nav::fsm
