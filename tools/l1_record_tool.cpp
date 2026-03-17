#include <cstdlib>
#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/drivers/lidar/l1_driver.hpp"
#include "rm_nav/utils/file_io.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string output_path{"logs/l1_record.rmr"};
  int duration_ms{1000};
  double frame_rate_hz{10.0};
  std::size_t points_per_frame{360};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--output" && index + 1 < argc) {
      options->output_path = argv[++index];
      continue;
    }
    if (arg == "--duration-ms" && index + 1 < argc) {
      options->duration_ms = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--frame-rate" && index + 1 < argc) {
      options->frame_rate_hz = std::atof(argv[++index]);
      continue;
    }
    if (arg == "--points-per-frame" && index + 1 < argc) {
      options->points_per_frame = static_cast<std::size_t>(std::atoi(argv[++index]));
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported l1_record_tool arg");
  }
  return rm_nav::common::Status::Ok();
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_record", status.message);
    return 1;
  }

  rm_nav::drivers::lidar::L1Driver driver;
  rm_nav::drivers::lidar::L1DriverConfig config;
  config.frame_rate_hz = options.frame_rate_hz;
  config.points_per_frame = options.points_per_frame;
  status = driver.Configure(config);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_record", status.message);
    return 1;
  }

  rm_nav::utils::RecorderWriter writer;
  status = writer.Open(options.output_path);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_record", status.message);
    return 1;
  }

  const auto start = rm_nav::common::Now();
  std::size_t frame_count = 0;
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    auto frame = driver.PollFrame();
    if (!frame.has_value()) {
      continue;
    }
    status = writer.WriteLidarFrame(*frame);
    if (!status.ok()) {
      rm_nav::utils::LogError("l1_record", status.message);
      writer.Close();
      return 1;
    }
    ++frame_count;
  }

  writer.Close();
  rm_nav::utils::LogInfo("l1_record",
                         "recorded lidar frames=" + std::to_string(frame_count) +
                             " file=" + options.output_path);
  return 0;
}
