#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

namespace rm_nav::common {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;
using WallClock = std::chrono::system_clock;
using WallTimePoint = WallClock::time_point;
using TimeNs = std::int64_t;

inline constexpr TimeNs kNanosecondsPerSecond = 1000000000LL;

inline TimePoint Now() { return Clock::now(); }
inline TimeNs NowNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Now().time_since_epoch()).count(); }
WallTimePoint WallNow();
std::string FormatWallTime(WallTimePoint time_point);
TimeNs ToNanoseconds(TimePoint time_point);
inline TimeNs ToNanoseconds(Duration duration) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}
TimePoint FromNanoseconds(TimeNs nanoseconds);
void SleepFor(std::chrono::milliseconds duration);
void SleepUntil(TimePoint time_point);

class TimeConverter {
 public:
  void Calibrate(TimeNs driver_time_ns, TimePoint system_time);
  void Reset();
  bool is_calibrated() const;
  TimePoint DriverToSystem(TimeNs driver_time_ns) const;
  TimeNs SystemToDriver(TimePoint system_time) const;

 private:
  std::atomic<TimeNs> offset_ns_{0};
  std::atomic<bool> calibrated_{false};
};

}  // namespace rm_nav::common
