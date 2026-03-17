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
  snapshot = fsm.Update(rm_nav::common::Now(), context);
  assert(snapshot.state == rm_nav::fsm::NavState::kRecovery);

  context.localization_degraded = false;
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

  std::cout << "test_nav_fsm passed\n";
  return 0;
}
