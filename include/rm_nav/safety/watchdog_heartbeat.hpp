#pragma once

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/safety_config.hpp"

namespace rm_nav::safety {

class WatchdogHeartbeat {
 public:
  bool IsAlive(common::TimeNs last_rx_ns, common::TimeNs now_ns,
               const config::SafetyConfig& config) const;
  bool IsAlive(common::TimePoint last_rx_stamp, common::TimePoint now,
               const config::SafetyConfig& config) const;
};

}  // namespace rm_nav::safety
