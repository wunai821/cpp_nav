#pragma once

#include "rm_nav/config/safety_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"

namespace rm_nav::safety {

class DeadmanChecker {
 public:
  bool PlannerCmdTimedOut(const data::ChassisCmd& cmd, common::TimePoint now,
                          const config::SafetyConfig& config) const;
};

}  // namespace rm_nav::safety
