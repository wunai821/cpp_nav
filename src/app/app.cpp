#include "rm_nav/app/app.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "rm_nav/utils/logger.hpp"

namespace rm_nav::app {
namespace {

std::atomic_bool g_stop_requested{false};

void HandleSignal(int) { g_stop_requested.store(true); }

void HandleCrashSignal(int signal_number) {
  g_stop_requested.store(true);
  ::mkdir("logs", 0755);
  ::mkdir("logs/crash", 0755);
  const int fd = ::open("logs/crash/signal.log", O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd >= 0) {
    char buffer[128];
    const int size = std::snprintf(buffer, sizeof(buffer),
                                   "fatal_signal=%d message=runtime crashed\n", signal_number);
    if (size > 0) {
      const auto write_size = static_cast<std::size_t>(std::min(size, static_cast<int>(sizeof(buffer))));
      ::write(fd, buffer, write_size);
    }
    ::close(fd);
  }
  std::_Exit(128 + signal_number);
}

common::Status ParseArgs(int argc, char** argv, std::string* config_dir,
                         bool* print_version) {
  if (config_dir == nullptr || print_version == nullptr) {
    return common::Status::InvalidArgument("argument output pointer is null");
  }

  *config_dir = "config";
  *print_version = false;

  const std::vector<std::string> args(argv, argv + argc);
  for (std::size_t index = 1; index < args.size(); ++index) {
    if (args[index] == "--config") {
      if (index + 1U >= args.size()) {
        return common::Status::InvalidArgument("missing value for --config");
      }
      *config_dir = args[index + 1U];
      ++index;
      continue;
    }
    if (args[index] == "--version") {
      *print_version = true;
      continue;
    }
    return common::Status::InvalidArgument("unsupported command line argument");
  }

  return common::Status::Ok();
}

}  // namespace

int App::Run(int argc, char** argv) {
  std::string config_dir;
  bool print_version = false;

  auto status = ParseArgs(argc, argv, &config_dir, &print_version);
  if (!status.ok()) {
    utils::Logger::Instance().Initialize(
        {utils::LogLevel::kInfo, "logs/rm_nav.log", true, false, 4096});
    utils::LogError("main", status.message);
    utils::Logger::Instance().Shutdown();
    return 1;
  }

  utils::Logger::Instance().Initialize(
      {utils::LogLevel::kInfo, "logs/rm_nav.log", true, false, 4096});

  if (print_version) {
    utils::LogInfo("main", std::string("rm_nav_main version ") + RM_NAV_VERSION);
    utils::Logger::Instance().Shutdown();
    return 0;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  std::signal(SIGSEGV, HandleCrashSignal);
  std::signal(SIGABRT, HandleCrashSignal);
  std::signal(SIGBUS, HandleCrashSignal);
  std::signal(SIGILL, HandleCrashSignal);

  std::filesystem::create_directories("logs/crash");
  std::filesystem::create_directories("logs/watchdog");

  Runtime runtime;
  status = runtime.Initialize(config_dir);
  if (!status.ok()) {
    utils::LogError("main", status.message);
    utils::Logger::Instance().Shutdown();
    return 1;
  }

  const int exit_code = runtime.Run(g_stop_requested);
  utils::LogInfo("main", "runtime exited cleanly");
  utils::Logger::Instance().Shutdown();
  return exit_code;
}

}  // namespace rm_nav::app
