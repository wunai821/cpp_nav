#include "rm_nav/fsm/mode_recovery.hpp"

namespace rm_nav::fsm {

const char* ToString(RecoveryTier tier) {
  switch (tier) {
    case RecoveryTier::kLight:
      return "L1_LIGHT";
    case RecoveryTier::kMedium:
      return "L2_MEDIUM";
    case RecoveryTier::kHeavy:
      return "L3_HEAVY";
    case RecoveryTier::kNone:
    default:
      return "NONE";
  }
}

const char* ToString(RecoveryCause cause) {
  switch (cause) {
    case RecoveryCause::kDynamicBlock:
      return "dynamic_block";
    case RecoveryCause::kCostmapCongested:
      return "costmap_congested";
    case RecoveryCause::kLocalPlannerStall:
      return "local_planner_stall";
    case RecoveryCause::kStuckNearGoal:
      return "stuck_near_goal";
    case RecoveryCause::kLocalizationLost:
      return "localization_lost";
    case RecoveryCause::kMapToOdomRejected:
      return "map_to_odom_rejected";
    case RecoveryCause::kRecoveryExhausted:
      return "recovery_exhausted";
    case RecoveryCause::kNone:
    default:
      return "none";
  }
}

const char* ToString(RecoveryAction action) {
  switch (action) {
    case RecoveryAction::kSlowdownResample:
      return "slowdown_resample";
    case RecoveryAction::kRotateInPlace:
      return "rotate_in_place";
    case RecoveryAction::kMicroSidestep:
      return "micro_sidestep";
    case RecoveryAction::kBackoff:
      return "backoff";
    case RecoveryAction::kEscapeSidestep:
      return "escape_sidestep";
    case RecoveryAction::kReapproachWaypoint:
      return "reapproach_waypoint";
    case RecoveryAction::kStopAndRelocalize:
      return "stop_and_relocalize";
    case RecoveryAction::kSlowRestart:
      return "slow_restart";
    case RecoveryAction::kNone:
    default:
      return "none";
  }
}

RecoveryTier MinimumTierForCause(RecoveryCause cause) {
  switch (cause) {
    case RecoveryCause::kDynamicBlock:
    case RecoveryCause::kCostmapCongested:
      return RecoveryTier::kLight;
    case RecoveryCause::kLocalPlannerStall:
    case RecoveryCause::kStuckNearGoal:
      return RecoveryTier::kMedium;
    case RecoveryCause::kLocalizationLost:
    case RecoveryCause::kMapToOdomRejected:
    case RecoveryCause::kRecoveryExhausted:
      return RecoveryTier::kHeavy;
    case RecoveryCause::kNone:
    default:
      return RecoveryTier::kNone;
  }
}

bool CauseNeedsRelocalization(RecoveryCause cause) {
  return cause == RecoveryCause::kLocalizationLost ||
         cause == RecoveryCause::kMapToOdomRejected ||
         cause == RecoveryCause::kRecoveryExhausted;
}

}  // namespace rm_nav::fsm
