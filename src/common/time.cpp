#include "rm_nav/common/time.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

namespace rm_nav::common {

WallTimePoint WallNow() { return WallClock::now(); }

std::string FormatWallTime(WallTimePoint time_point) {
  const auto time_t = WallClock::to_time_t(time_point);
  const auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          time_point.time_since_epoch()) %
      1000;

  std::tm tm_value{};
#if defined(_WIN32)
  localtime_s(&tm_value, &time_t);
#else
  localtime_r(&time_t, &tm_value);
#endif

  std::ostringstream stream;
  stream << std::put_time(&tm_value, "%Y-%m-%d %H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << milliseconds.count();
  return stream.str();
}

TimeNs ToNanoseconds(TimePoint time_point) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             time_point.time_since_epoch())
      .count();
}

TimePoint FromNanoseconds(TimeNs nanoseconds) {
  return TimePoint(std::chrono::nanoseconds(nanoseconds));
}

void SleepFor(std::chrono::milliseconds duration) {
  std::this_thread::sleep_for(duration);
}

void SleepUntil(TimePoint time_point) { std::this_thread::sleep_until(time_point); }

void TimeConverter::Calibrate(TimeNs driver_time_ns, TimePoint system_time) {
  offset_ns_.store(ToNanoseconds(system_time) - driver_time_ns,
                   std::memory_order_release);
  calibrated_.store(true, std::memory_order_release);
}

void TimeConverter::Reset() {
  offset_ns_.store(0, std::memory_order_release);
  calibrated_.store(false, std::memory_order_release);
}

bool TimeConverter::is_calibrated() const {
  return calibrated_.load(std::memory_order_acquire);
}

TimePoint TimeConverter::DriverToSystem(TimeNs driver_time_ns) const {
  return FromNanoseconds(driver_time_ns + offset_ns_.load(std::memory_order_acquire));
}

TimeNs TimeConverter::SystemToDriver(TimePoint system_time) const {
  return ToNanoseconds(system_time) - offset_ns_.load(std::memory_order_acquire);
}

}  // namespace rm_nav::common
