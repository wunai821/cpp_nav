#include "rm_nav/utils/logger.hpp"

#include <iostream>
#include <sstream>
#include <thread>

#include "rm_nav/common/time.hpp"

namespace rm_nav::utils {
namespace {

std::string BuildLogLine(LogLevel level, std::string_view component,
                         std::string_view message) {
  std::ostringstream stream;
  stream << '[' << rm_nav::common::FormatWallTime(rm_nav::common::WallNow()) << ']'
         << '[';
  switch (level) {
    case LogLevel::kDebug:
      stream << "DEBUG";
      break;
    case LogLevel::kInfo:
      stream << "INFO";
      break;
    case LogLevel::kWarn:
      stream << "WARN";
      break;
    case LogLevel::kError:
      stream << "ERROR";
      break;
  }
  stream << "][" << component << "][tid=" << std::this_thread::get_id() << "] "
         << message;
  return stream.str();
}

}  // namespace

Logger& Logger::Instance() {
  static Logger logger;
  return logger;
}

void Logger::Initialize(const LoggerOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  level_ = options.level;
  initialized_ = true;
}

void Logger::Log(LogLevel level, std::string_view component,
                 std::string_view message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !ShouldLog(level)) {
    return;
  }
  std::cout << BuildLogLine(level, component, message) << std::endl;
}

LogLevel Logger::level() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return level_;
}

bool Logger::ShouldLog(LogLevel level) const {
  return static_cast<int>(level) >= static_cast<int>(level_);
}

LogLevel ParseLogLevel(std::string_view value) {
  if (value == "DEBUG") {
    return LogLevel::kDebug;
  }
  if (value == "WARN") {
    return LogLevel::kWarn;
  }
  if (value == "ERROR") {
    return LogLevel::kError;
  }
  return LogLevel::kInfo;
}

void LogDebug(std::string_view component, std::string_view message) {
  Logger::Instance().Log(LogLevel::kDebug, component, message);
}

void LogInfo(std::string_view component, std::string_view message) {
  Logger::Instance().Log(LogLevel::kInfo, component, message);
}

void LogWarn(std::string_view component, std::string_view message) {
  Logger::Instance().Log(LogLevel::kWarn, component, message);
}

void LogError(std::string_view component, std::string_view message) {
  Logger::Instance().Log(LogLevel::kError, component, message);
}

}  // namespace rm_nav::utils
