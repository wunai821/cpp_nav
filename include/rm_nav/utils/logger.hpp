#pragma once

#include <mutex>
#include <string_view>

namespace rm_nav::utils {

enum class LogLevel {
  kDebug = 0,
  kInfo,
  kWarn,
  kError,
};

struct LoggerOptions {
  LogLevel level{LogLevel::kInfo};
};

class Logger {
 public:
  static Logger& Instance();

  void Initialize(const LoggerOptions& options);
  void Log(LogLevel level, std::string_view component, std::string_view message);
  LogLevel level() const;

 private:
  Logger() = default;

  bool ShouldLog(LogLevel level) const;

  mutable std::mutex mutex_{};
  LogLevel level_{LogLevel::kInfo};
  bool initialized_{false};
};

LogLevel ParseLogLevel(std::string_view value);
void LogDebug(std::string_view component, std::string_view message);
void LogInfo(std::string_view component, std::string_view message);
void LogWarn(std::string_view component, std::string_view message);
void LogError(std::string_view component, std::string_view message);

}  // namespace rm_nav::utils
