#pragma once

namespace rm_nav::fsm {

enum class RecoveryTier {
  kNone = 0,
  kLight,
  kMedium,
  kHeavy,
};

enum class RecoveryCause {
  kNone = 0,
  kDynamicBlock,
  kCostmapCongested,
  kLocalPlannerStall,
  kStuckNearGoal,
  kLocalizationLost,
  kMapToOdomRejected,
  kRecoveryExhausted,
};

enum class RecoveryAction {
  kNone = 0,
  kSlowdownResample,
  kRotateInPlace,
  kMicroSidestep,
  kBackoff,
  kEscapeSidestep,
  kReapproachWaypoint,
  kStopAndRelocalize,
  kSlowRestart,
};

const char* ToString(RecoveryTier tier);
const char* ToString(RecoveryCause cause);
const char* ToString(RecoveryAction action);
RecoveryTier MinimumTierForCause(RecoveryCause cause);
bool CauseNeedsRelocalization(RecoveryCause cause);

}  // namespace rm_nav::fsm
