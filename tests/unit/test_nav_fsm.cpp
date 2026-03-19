#include <cassert>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/fsm/nav_fsm.hpp"

int main() {
  rm_nav::fsm::NavFsm fsm;
  rm_nav::fsm::NavFsmContext context;
  context.mapping_enabled = false;
  context.map_loaded = true;
  context.combat_ready = true;

  auto snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kSelfCheck);
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kIdle);
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeCombat);
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kGotoCenter);

  context.center_reached = true;
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kHoldCenter);

  context.center_reached = false;
  context.center_drifted = true;
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kRecenter);

  context.center_drifted = false;
  context.localization_degraded = true;
  context.recovery_tier = rm_nav::fsm::RecoveryTier::kHeavy;
  context.recovery_action = rm_nav::fsm::RecoveryAction::kStopAndRelocalize;
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kRecovery);
  assert(snapshot.recovery_tier == rm_nav::fsm::RecoveryTier::kHeavy);
  assert(snapshot.recovery_action == rm_nav::fsm::RecoveryAction::kStopAndRelocalize);

  context.localization_degraded = false;
  context.recovery_complete = true;
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kGotoCenter);

  context.safety_triggered = true;
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kFailsafe);

  rm_nav::fsm::NavFsm warmup_fsm;
  rm_nav::fsm::NavFsmContext warmup_context;
  warmup_context.mapping_enabled = true;

  snapshot = warmup_fsm.Update(rm_nav::common::Now(), warmup_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kSelfCheck);
  snapshot = warmup_fsm.Update(rm_nav::common::Now(), warmup_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kIdle);
  snapshot = warmup_fsm.Update(rm_nav::common::Now(), warmup_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeWarmup);

  warmup_context.save_requested = true;
  snapshot = warmup_fsm.Update(rm_nav::common::Now(), warmup_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeSave);

  warmup_context.mapping_enabled = false;
  warmup_context.save_requested = false;
  warmup_context.map_saved = true;
  warmup_context.map_loaded = true;
  warmup_context.combat_ready = true;
  snapshot = warmup_fsm.Update(rm_nav::common::Now(), warmup_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeCombat);

  rm_nav::fsm::NavFsm save_failure_fsm;
  rm_nav::fsm::NavFsmContext save_failure_context;
  save_failure_context.mapping_enabled = true;
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeWarmup);
  save_failure_context.save_requested = true;
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeSave);

  save_failure_context.save_requested = false;
  save_failure_context.map_save_validation_failed = true;
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeSave);
  assert(snapshot.last_event.code == rm_nav::fsm::NavEventCode::kMapValidationFailed);

  save_failure_context.map_save_validation_failed = false;
  save_failure_context.map_save_storage_failed = true;
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  assert(snapshot.last_event.code == rm_nav::fsm::NavEventCode::kMapStorageSwitchFailed);

  save_failure_context.map_save_storage_failed = false;
  save_failure_context.map_save_write_failed = true;
  snapshot = save_failure_fsm.Update(rm_nav::common::Now(), save_failure_context);
  assert(snapshot.last_event.code == rm_nav::fsm::NavEventCode::kMapSaveWriteFailed);

  rm_nav::fsm::NavFsm fallback_fsm;
  rm_nav::fsm::NavFsmContext fallback_context;
  fallback_context.mapping_enabled = true;
  snapshot = fallback_fsm.Update(rm_nav::common::Now(), fallback_context);
  snapshot = fallback_fsm.Update(rm_nav::common::Now(), fallback_context);
  snapshot = fallback_fsm.Update(rm_nav::common::Now(), fallback_context);
  fallback_context.save_requested = true;
  snapshot = fallback_fsm.Update(rm_nav::common::Now(), fallback_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeSave);
  fallback_context.save_requested = false;
  fallback_context.combat_ready = true;
  fallback_context.map_loaded = true;
  snapshot = fallback_fsm.Update(rm_nav::common::Now(), fallback_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kModeCombat);
  assert(snapshot.last_event.code == rm_nav::fsm::NavEventCode::kMapLoadSuccess);

  rm_nav::fsm::NavFsm no_map_fsm;
  rm_nav::fsm::NavFsmContext no_map_context;
  no_map_context.combat_requested = true;
  no_map_context.map_unavailable = true;
  snapshot = no_map_fsm.Update(rm_nav::common::Now(), no_map_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kSelfCheck);
  snapshot = no_map_fsm.Update(rm_nav::common::Now(), no_map_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kIdle);
  snapshot = no_map_fsm.Update(rm_nav::common::Now(), no_map_context);
  assert(snapshot.state == rm_nav::fsm::NavState::kFailsafe);
  assert(snapshot.last_event.code == rm_nav::fsm::NavEventCode::kMapUnavailable);

  std::cout << "test_nav_fsm passed\n";
  return 0;
}
