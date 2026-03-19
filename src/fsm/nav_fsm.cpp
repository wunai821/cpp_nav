#include "rm_nav/fsm/nav_fsm.hpp"

#include <sstream>

#include "rm_nav/utils/logger.hpp"

namespace rm_nav::fsm {
const char* ToString(NavState state);
const char* ToString(NavEventCode code);

namespace {
void LogFsmStatus(const NavFsmSnapshot& snapshot, const NavFsmContext& context) {
  std::ostringstream stream;
  stream << "mode=" << ToString(snapshot.state)
         << " prev=" << ToString(snapshot.previous_state)
         << " event=" << ToString(snapshot.last_event.code)
         << " map=" << context.map_label
         << " matcher=" << context.localization_matcher
         << " recovery_tier=" << ToString(context.recovery_tier)
         << " recovery_cause=" << ToString(context.recovery_cause)
         << " recovery_action=" << ToString(context.recovery_action)
         << " recovery_strategy=" << context.recovery_strategy
         << " degrade=" << context.degraded_mode
         << " loc_reason=" << context.localization_reason
         << " planner_reason=" << context.planner_reason
         << " safety_reason=" << context.safety_reason;
  if (snapshot.state == NavState::kRecovery || snapshot.state == NavState::kFailsafe) {
    utils::LogWarn("fsm", stream.str());
  } else {
    utils::LogInfo("fsm", stream.str());
  }
}

}  // namespace

const char* ToString(NavState state) {
  switch (state) {
    case NavState::kBoot:
      return "BOOT";
    case NavState::kSelfCheck:
      return "SELF_CHECK";
    case NavState::kIdle:
      return "IDLE";
    case NavState::kModeWarmup:
      return "MODE_WARMUP";
    case NavState::kModeSave:
      return "MODE_SAVE";
    case NavState::kModeCombat:
      return "MODE_COMBAT";
    case NavState::kGotoCenter:
      return "GOTO_CENTER";
    case NavState::kHoldCenter:
      return "HOLD_CENTER";
    case NavState::kRecenter:
      return "RECENTER";
    case NavState::kRecovery:
      return "RECOVERY";
    case NavState::kFailsafe:
      return "FAILSAFE";
  }
  return "UNKNOWN";
}

const char* ToString(NavEventCode code) {
  switch (code) {
    case NavEventCode::kNone:
      return "NONE";
    case NavEventCode::kSelfCheckPassed:
      return "SELF_CHECK_PASSED";
    case NavEventCode::kEnterWarmupMode:
      return "ENTER_WARMUP_MODE";
    case NavEventCode::kEnterCombatMode:
      return "ENTER_COMBAT_MODE";
    case NavEventCode::kMapLoadSuccess:
      return "MAP_LOAD_SUCCESS";
    case NavEventCode::kMapSaveSuccess:
      return "MAP_SAVE_SUCCESS";
    case NavEventCode::kMapSaveWriteFailed:
      return "MAP_SAVE_WRITE_FAILED";
    case NavEventCode::kMapValidationFailed:
      return "MAP_VALIDATION_FAILED";
    case NavEventCode::kMapStorageSwitchFailed:
      return "MAP_STORAGE_SWITCH_FAILED";
    case NavEventCode::kMapUnavailable:
      return "MAP_UNAVAILABLE";
    case NavEventCode::kLocalizationDegraded:
      return "LOCALIZATION_DEGRADED";
    case NavEventCode::kPlannerFailed:
      return "PLANNER_FAILED";
    case NavEventCode::kCenterDrifted:
      return "CENTER_DRIFTED";
    case NavEventCode::kSafetyTriggered:
      return "SAFETY_TRIGGERED";
    case NavEventCode::kHeartbeatAnomaly:
      return "HEARTBEAT_ANOMALY";
    case NavEventCode::kRefereeStateChanged:
      return "REFEREE_STATE_CHANGED";
    case NavEventCode::kRecoveryComplete:
      return "RECOVERY_COMPLETE";
    case NavEventCode::kRecoveryEscalated:
      return "RECOVERY_ESCALATED";
  }
  return "UNKNOWN";
}

NavEvent NavFsm::MakeEvent(common::TimePoint stamp, NavEventCode code,
                           const char* summary) const {
  NavEvent event;
  event.stamp = stamp;
  event.code = code;
  event.summary = summary;
  return event;
}

void NavFsm::Transition(common::TimePoint stamp, NavState next_state, NavEventCode event_code,
                        const char* summary) {
  snapshot_.previous_state = snapshot_.state;
  snapshot_.state = next_state;
  snapshot_.last_event = MakeEvent(stamp, event_code, summary);
  RefreshFlags();
}

void NavFsm::RefreshFlags() {
  snapshot_.mapping_active = snapshot_.state == NavState::kModeWarmup;
  snapshot_.waypoint_active = snapshot_.state == NavState::kModeWarmup;
  snapshot_.localization_active =
      snapshot_.state == NavState::kModeCombat || snapshot_.state == NavState::kGotoCenter ||
      snapshot_.state == NavState::kHoldCenter || snapshot_.state == NavState::kRecenter ||
      snapshot_.state == NavState::kRecovery;
  snapshot_.center_hold_active = snapshot_.state == NavState::kHoldCenter;
  snapshot_.recovery_active = snapshot_.state == NavState::kRecovery;
  snapshot_.failsafe_active = snapshot_.state == NavState::kFailsafe;
  snapshot_.recovery_tier =
      snapshot_.state == NavState::kRecovery ? snapshot_.recovery_tier : RecoveryTier::kNone;
  snapshot_.recovery_action =
      snapshot_.state == NavState::kRecovery ? snapshot_.recovery_action : RecoveryAction::kNone;
}

NavFsmSnapshot NavFsm::Update(common::TimePoint stamp, const NavFsmContext& context) {
  const auto state_before = snapshot_.state;
  const auto event_before = snapshot_.last_event.code;
  if (context.referee_changed) {
    snapshot_.last_event =
        MakeEvent(stamp, NavEventCode::kRefereeStateChanged, "referee state changed");
  }

  if (!context.heartbeat_ok) {
    Transition(stamp, NavState::kFailsafe, NavEventCode::kHeartbeatAnomaly,
               "heartbeat anomaly");
    LogFsmStatus(snapshot_, context);
    return snapshot_;
  }
  if (context.safety_triggered) {
    Transition(stamp, NavState::kFailsafe, NavEventCode::kSafetyTriggered,
               "safety trigger");
    LogFsmStatus(snapshot_, context);
    return snapshot_;
  }

  switch (snapshot_.state) {
    case NavState::kBoot:
      Transition(stamp, NavState::kSelfCheck, NavEventCode::kNone, "boot complete");
      break;
    case NavState::kSelfCheck:
      Transition(stamp, NavState::kIdle, NavEventCode::kSelfCheckPassed, "self check passed");
      break;
    case NavState::kIdle:
      if (context.combat_requested && context.map_unavailable) {
        Transition(stamp, NavState::kFailsafe, NavEventCode::kMapUnavailable,
                   "no usable map available");
      } else if (context.mapping_enabled) {
        Transition(stamp, NavState::kModeWarmup, NavEventCode::kEnterWarmupMode,
                   "enter warmup mode");
      } else if (context.combat_ready && context.map_loaded) {
        Transition(stamp, NavState::kModeCombat, NavEventCode::kEnterCombatMode,
                   "enter combat mode");
      }
      break;
    case NavState::kModeWarmup:
      if (context.save_requested) {
        Transition(stamp, NavState::kModeSave, NavEventCode::kNone, "save requested");
      }
      break;
    case NavState::kModeSave:
      if (context.map_save_write_failed) {
        snapshot_.last_event =
            MakeEvent(stamp, NavEventCode::kMapSaveWriteFailed, "map save write failed");
      } else if (context.map_save_validation_failed) {
        snapshot_.last_event =
            MakeEvent(stamp, NavEventCode::kMapValidationFailed, "map validation failed");
      } else if (context.map_save_storage_failed) {
        snapshot_.last_event = MakeEvent(stamp, NavEventCode::kMapStorageSwitchFailed,
                                         "map storage switch failed");
      } else if (context.combat_ready && context.map_loaded) {
        Transition(stamp, NavState::kModeCombat, NavEventCode::kMapLoadSuccess,
                   "fallback map load success");
      } else if (context.map_unavailable) {
        Transition(stamp, NavState::kIdle, NavEventCode::kMapUnavailable,
                   "no usable map available");
      } else if (context.map_saved) {
        Transition(stamp,
                   (context.combat_ready && context.map_loaded) ? NavState::kModeCombat
                                                                : NavState::kIdle,
                   NavEventCode::kMapSaveSuccess, "map save success");
      }
      break;
    case NavState::kModeCombat:
      if (context.localization_degraded) {
        Transition(stamp, NavState::kRecovery, NavEventCode::kLocalizationDegraded,
                   "localization degraded");
      } else if (context.map_loaded) {
        Transition(stamp, NavState::kGotoCenter, NavEventCode::kMapLoadSuccess,
                   "map load success");
      }
      break;
    case NavState::kGotoCenter:
      if (context.localization_degraded || context.planner_failed) {
        Transition(stamp, NavState::kRecovery,
                   context.localization_degraded ? NavEventCode::kLocalizationDegraded
                                                 : NavEventCode::kPlannerFailed,
                   context.localization_degraded ? "localization degraded"
                                                 : "planner failed");
      } else if (context.center_reached) {
        Transition(stamp, NavState::kHoldCenter, NavEventCode::kNone, "center reached");
      }
      break;
    case NavState::kHoldCenter:
      if (context.localization_degraded) {
        Transition(stamp, NavState::kRecovery, NavEventCode::kLocalizationDegraded,
                   "localization degraded");
      } else if (context.center_drifted) {
        Transition(stamp, NavState::kRecenter, NavEventCode::kCenterDrifted,
                   "center drifted");
      }
      break;
    case NavState::kRecenter:
      if (context.localization_degraded || context.planner_failed) {
        Transition(stamp, NavState::kRecovery,
                   context.localization_degraded ? NavEventCode::kLocalizationDegraded
                                                 : NavEventCode::kPlannerFailed,
                   context.localization_degraded ? "localization degraded"
                                                 : "planner failed");
      } else if (context.center_reached) {
        Transition(stamp, NavState::kHoldCenter, NavEventCode::kNone, "recentering complete");
      }
      break;
    case NavState::kRecovery:
      snapshot_.recovery_tier = context.recovery_tier;
      snapshot_.recovery_action = context.recovery_action;
      if (context.recovery_exhausted) {
        Transition(stamp, NavState::kFailsafe, NavEventCode::kRecoveryEscalated,
                   "recovery exhausted");
      } else if (context.recovery_complete && context.combat_ready && context.map_loaded) {
        Transition(stamp, NavState::kGotoCenter, NavEventCode::kRecoveryComplete,
                   "recovery complete");
      }
      break;
    case NavState::kFailsafe:
      if (context.heartbeat_ok && !context.safety_triggered && !context.map_unavailable) {
        Transition(stamp,
                   context.mapping_enabled ? NavState::kModeWarmup
                                           : (context.combat_ready && context.map_loaded
                                                  ? NavState::kModeCombat
                                                  : NavState::kIdle),
                   NavEventCode::kNone, "failsafe released");
      }
      break;
  }

  RefreshFlags();
  if (snapshot_.state != NavState::kRecovery) {
    snapshot_.recovery_tier = RecoveryTier::kNone;
    snapshot_.recovery_action = RecoveryAction::kNone;
  } else {
    snapshot_.recovery_tier = context.recovery_tier;
    snapshot_.recovery_action = context.recovery_action;
  }
  if (snapshot_.state != state_before || snapshot_.last_event.code != event_before) {
    LogFsmStatus(snapshot_, context);
  }
  return snapshot_;
}

}  // namespace rm_nav::fsm
