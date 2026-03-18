#include "rm_nav/safety/watchdog_heartbeat.hpp"

namespace rm_nav::safety {

bool WatchdogHeartbeat::IsAlive(common::TimeNs last_rx_ns, common::TimeNs now_ns,
                                const config::SafetyConfig& config) const {
  if (last_rx_ns <= 0) {
    return false;
  }
  return (now_ns - last_rx_ns) <=
         static_cast<common::TimeNs>(config.heartbeat_timeout_ms) * 1000000LL;
}

bool WatchdogHeartbeat::IsAlive(common::TimePoint last_rx_stamp, common::TimePoint now,
                                const config::SafetyConfig& config) const {
  if (last_rx_stamp == common::TimePoint{}) {
    return false;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_stamp).count() <=
         config.heartbeat_timeout_ms;
}

}  // namespace rm_nav::safety
