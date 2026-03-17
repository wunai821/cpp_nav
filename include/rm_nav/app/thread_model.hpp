#pragma once

#include <array>
#include <string_view>

namespace rm_nav::app {

enum class ThreadDomain {
  kDriver = 0,
  kSync,
  kPoseCore,
  kPerception,
  kPlanner,
  kSafetyFsm,
  kDebug,
};

struct ThreadBinding {
  ThreadDomain domain{};
  std::string_view display_name{};
};

inline constexpr std::array<ThreadBinding, 7> kThreadBindings = {{
    {ThreadDomain::kDriver, "Driver Thread"},
    {ThreadDomain::kSync, "Sync Thread"},
    {ThreadDomain::kPoseCore, "Pose Core Thread"},
    {ThreadDomain::kPerception, "Perception Thread"},
    {ThreadDomain::kPlanner, "Planner Thread"},
    {ThreadDomain::kSafetyFsm, "Safety/FSM Thread"},
    {ThreadDomain::kDebug, "Debug Thread"},
}};

}  // namespace rm_nav::app
