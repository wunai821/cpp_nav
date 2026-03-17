#pragma once

namespace rm_nav::fsm {

enum class NavState {
  kBoot = 0,
  kSelfCheck,
  kIdle,
  kModeWarmup,
  kModeSave,
  kModeCombat,
  kGotoCenter,
  kHoldCenter,
  kRecenter,
  kRecovery,
  kFailsafe,
};

const char* ToString(NavState state);

}  // namespace rm_nav::fsm
