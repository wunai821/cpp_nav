#include "rm_nav/utils/logger.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <ctime>
#include <iomanip>

#include "rm_nav/common/time.hpp"

namespace rm_nav::utils {
namespace {

const char* ToString(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "DEBUG";
    case LogLevel::kInfo:
      return "INFO";
    case LogLevel::kWarn:
      return "WARN";
    case LogLevel::kError:
      return "ERROR";
  }
  return "INFO";
}

std::string BaseName(std::string_view path) {
  const auto slash = path.find_last_of("/\\");
  if (slash == std::string_view::npos) {
    return std::string(path);
  }
  return std::string(path.substr(slash + 1U));
}

bool IsIoComponent(std::string_view component) {
  return component == "l1_driver" || component == "imu_driver" || component == "driver" ||
         component == "stm32" || component == "serial";
}

std::string DateToken(common::WallTimePoint time_point) {
  const auto time_t = common::WallClock::to_time_t(time_point);
  std::tm tm_value{};
#if defined(_WIN32)
  localtime_s(&tm_value, &time_t);
#else
  localtime_r(&time_t, &tm_value);
#endif

  std::ostringstream stream;
  stream << std::put_time(&tm_value, "%Y-%m-%d");
  return stream.str();
}

std::filesystem::path DatedLogPath(const std::string& base_path,
                                   std::string_view date_token) {
  const auto path = std::filesystem::path(base_path);
  const auto stem = path.stem().string();
  const auto extension = path.extension().string();
  const auto parent = path.parent_path();
  std::filesystem::path dated_name = stem + "-" + std::string(date_token) + extension;
  if (parent.empty()) {
    return dated_name;
  }
  return parent / dated_name;
}

std::string BuildConsoleLine(LogLevel level, std::string_view component,
                             std::string_view message) {
  std::ostringstream stream;
  stream << '[' << rm_nav::common::FormatWallTime(rm_nav::common::WallNow()) << ']'
         << '[' << ToString(level) << "][" << component << "][tid=" << std::this_thread::get_id()
         << "] " << message;
  return stream.str();
}

std::string BuildFileLine(LogLevel level, std::string_view component, std::string_view message,
                          const char* file, int line, const char* function) {
  std::ostringstream stream;
  stream << '[' << rm_nav::common::FormatWallTime(rm_nav::common::WallNow()) << ']'
         << '[' << ToString(level) << "][" << component << "][tid=" << std::this_thread::get_id()
         << "][func=" << (function != nullptr ? function : "unknown") << "][src="
         << BaseName(file != nullptr ? file : "unknown") << ':' << line << "] " << message;
  return stream.str();
}

}  // namespace

Logger::~Logger() { Shutdown(); }

Logger& Logger::Instance() {
  static Logger logger;
  return logger;
}

void Logger::Initialize(const LoggerOptions& options) {
  Shutdown();

  std::lock_guard<std::mutex> lock(mutex_);
  level_ = options.level;
  file_path_ = options.file_path;
  file_enabled_ = options.file_enabled;
  console_io_only_ = options.console_io_only;
  max_queue_size_ = std::max<std::size_t>(1U, options.max_queue_size);
  dropped_overflow_logs_ = 0;
  stop_requested_ = false;
  initialized_ = true;
  worker_ = std::thread([this, file_path = file_path_, file_enabled = file_enabled_]() {
    WorkerMain(file_path, file_enabled);
  });
}

void Logger::Shutdown() {
  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ && !worker_.joinable()) {
      return;
    }
    stop_requested_ = true;
    initialized_ = false;
    queue_cv_.notify_all();
    worker = std::move(worker_);
  }
  if (worker.joinable()) {
    worker.join();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    stop_requested_ = false;
  }
}

void Logger::Log(LogLevel level, std::string_view component, std::string_view message,
                 const char* file, int line, const char* function) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !ShouldLog(level)) {
    return;
  }

  PendingLogEntry entry;
  entry.level = level;
  entry.to_console = ShouldLogToConsole(component);
  entry.to_file = file_enabled_;
  if (entry.to_console) {
    entry.console_line = BuildConsoleLine(level, component, message);
  }
  if (entry.to_file) {
    entry.file_line = BuildFileLine(level, component, message, file, line, function);
  }
  if (EnqueueEntry(std::move(entry))) {
    queue_cv_.notify_one();
  }
}

LogLevel Logger::level() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return level_;
}

bool Logger::ShouldLog(LogLevel level) const {
  return static_cast<int>(level) >= static_cast<int>(level_);
}

bool Logger::ShouldLogToConsole(std::string_view component) const {
  if (!console_io_only_) {
    return true;
  }
  return IsIoComponent(component);
}

bool Logger::EnqueueEntry(PendingLogEntry entry) {
  if (queue_.size() < max_queue_size_) {
    queue_.push_back(std::move(entry));
    return true;
  }

  if (entry.level < LogLevel::kWarn) {
    ++dropped_overflow_logs_;
    return false;
  }

  for (auto it = queue_.begin(); it != queue_.end(); ++it) {
    if (it->level < LogLevel::kWarn) {
      queue_.erase(it);
      queue_.push_back(std::move(entry));
      ++dropped_overflow_logs_;
      return true;
    }
  }

  queue_.pop_front();
  queue_.push_back(std::move(entry));
  ++dropped_overflow_logs_;
  return true;
}

void Logger::WorkerMain(std::string file_path, bool file_enabled) {
  std::ofstream file_output;
  std::string current_date_token;
  std::filesystem::path current_file_path;
  if (file_enabled) {
    current_date_token = DateToken(common::WallNow());
    current_file_path = DatedLogPath(file_path, current_date_token);
    if (!current_file_path.parent_path().empty()) {
      std::filesystem::create_directories(current_file_path.parent_path());
    }
    file_output.open(current_file_path, std::ios::out | std::ios::app);
  }

  while (true) {
    PendingLogEntry entry;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      queue_cv_.wait(lock, [this]() { return stop_requested_ || !queue_.empty(); });
      if (queue_.empty() && stop_requested_) {
        break;
      }
      entry = std::move(queue_.front());
      queue_.pop_front();
    }

    if (entry.to_console) {
      std::cout << entry.console_line << '\n';
    }
    if (entry.to_file && file_output.is_open()) {
      const auto next_date_token = DateToken(common::WallNow());
      if (next_date_token != current_date_token) {
        file_output.flush();
        file_output.close();
        current_date_token = next_date_token;
        current_file_path = DatedLogPath(file_path, current_date_token);
        if (!current_file_path.parent_path().empty()) {
          std::filesystem::create_directories(current_file_path.parent_path());
        }
        file_output.open(current_file_path, std::ios::out | std::ios::app);
      }
      file_output << entry.file_line << '\n';
      if (entry.level >= LogLevel::kWarn) {
        file_output.flush();
      }
    }
  }

  if (file_output.is_open()) {
    std::uint64_t dropped_overflow_logs = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dropped_overflow_logs = dropped_overflow_logs_;
    }
    if (dropped_overflow_logs > 0) {
      file_output << '[' << rm_nav::common::FormatWallTime(rm_nav::common::WallNow()) << ']'
                  << "[WARN][logger][tid=" << std::this_thread::get_id() << "][func=WorkerMain][src="
                  << BaseName(__FILE__) << ':' << __LINE__ << "] dropped_overflow_logs="
                  << dropped_overflow_logs << '\n';
    }
    file_output.flush();
  }
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
