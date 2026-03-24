#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace rm_nav::utils {

enum class LogLevel {
  kDebug = 0,
  kInfo,
  kWarn,
  kError,
};

struct LoggerOptions {
  LogLevel level{LogLevel::kInfo};
  std::string file_path{"logs/rm_nav.log"};
  bool file_enabled{true};
  bool console_io_only{false};
  std::size_t max_queue_size{4096};
};

class Logger {
 public:
  static Logger& Instance();

  ~Logger();
  void Initialize(const LoggerOptions& options);
  void Shutdown();
  void Log(LogLevel level, std::string_view component, std::string_view message,
           const char* file = __builtin_FILE(),
           int line = __builtin_LINE(),
           const char* function = __builtin_FUNCTION());
  LogLevel level() const;

 private:
  struct PendingLogEntry {
    std::string console_line{};
    std::string file_line{};
    LogLevel level{LogLevel::kInfo};
    bool to_console{false};
    bool to_file{false};
  };

  Logger() = default;

  bool ShouldLog(LogLevel level) const;
  bool ShouldLogToConsole(std::string_view component) const;
  bool EnqueueEntry(PendingLogEntry entry);
  void WorkerMain(std::string file_path, bool file_enabled);

  mutable std::mutex mutex_{};
  std::condition_variable queue_cv_{};
  std::deque<PendingLogEntry> queue_{};
  std::thread worker_{};
  LogLevel level_{LogLevel::kInfo};
  std::string file_path_{"logs/rm_nav.log"};
  bool file_enabled_{true};
  bool console_io_only_{false};
  std::size_t max_queue_size_{4096};
  std::uint64_t dropped_overflow_logs_{0};
  bool stop_requested_{false};
  bool initialized_{false};
};

LogLevel ParseLogLevel(std::string_view value);
void LogDebug(std::string_view component, std::string_view message);
void LogInfo(std::string_view component, std::string_view message);
void LogWarn(std::string_view component, std::string_view message);
void LogError(std::string_view component, std::string_view message);

}  // namespace rm_nav::utils
