#include "rm_nav/safety/deadman_checker.hpp"

namespace rm_nav::safety {

bool DeadmanChecker::PlannerCmdTimedOut(const data::ChassisCmd& cmd, common::TimePoint now,
                                        const config::SafetyConfig& config) const {
  if (cmd.stamp == common::TimePoint{}) {
    return true;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - cmd.stamp).count() >
         config.deadman_timeout_ms;
}

}  // namespace rm_nav::safety
