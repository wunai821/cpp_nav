#pragma once

#include "rm_nav/fsm/nav_event.hpp"
#include "rm_nav/fsm/nav_state.hpp"

namespace rm_nav::fsm {

struct NavFsmContext {
  bool mapping_enabled{false};
  bool save_requested{false};
  bool map_loaded{false};
  bool map_saved{false};
  bool map_save_write_failed{false};
  bool map_save_validation_failed{false};
  bool map_save_storage_failed{false};
  bool combat_requested{false};
  bool map_unavailable{false};
  bool combat_ready{false};
  bool localization_degraded{false};
  bool planner_failed{false};
  bool center_reached{false};
  bool center_drifted{false};
  bool safety_triggered{false};
  bool heartbeat_ok{true};
  bool referee_changed{false};
};

struct NavFsmSnapshot {
  NavState state{NavState::kBoot};
  NavState previous_state{NavState::kBoot};
  NavEvent last_event{};
  bool mapping_active{false};
  bool waypoint_active{false};
  bool localization_active{false};
  bool center_hold_active{false};
  bool recovery_active{false};
  bool failsafe_active{false};
};

class NavFsm {
 public:
  NavFsm() = default;

  NavFsmSnapshot Update(common::TimePoint stamp, const NavFsmContext& context);
  NavFsmSnapshot snapshot() const { return snapshot_; }

 private:
  NavEvent MakeEvent(common::TimePoint stamp, NavEventCode code,
                     const char* summary) const;
  void Transition(common::TimePoint stamp, NavState next_state, NavEventCode event_code,
                  const char* summary);
  void RefreshFlags();

  NavFsmSnapshot snapshot_{};
};

}  // namespace rm_nav::fsm
