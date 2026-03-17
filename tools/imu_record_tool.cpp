#include <cstdlib>
#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/drivers/imu/imu_driver.hpp"
#include "rm_nav/utils/file_io.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string output_path{"logs/imu_record.rmr"};
  int duration_ms{1000};
  double sample_rate_hz{200.0};
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
    if (arg == "--sample-rate" && index + 1 < argc) {
      options->sample_rate_hz = std::atof(argv[++index]);
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported imu_record_tool arg");
  }
  return rm_nav::common::Status::Ok();
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("imu_record", status.message);
    return 1;
  }

  rm_nav::drivers::imu::ImuDriver driver;
  rm_nav::drivers::imu::ImuDriverConfig config;
  config.sample_rate_hz = options.sample_rate_hz;
  status = driver.Configure(config);
  if (!status.ok()) {
    rm_nav::utils::LogError("imu_record", status.message);
    return 1;
  }

  rm_nav::utils::RecorderWriter writer;
  status = writer.Open(options.output_path);
  if (!status.ok()) {
    rm_nav::utils::LogError("imu_record", status.message);
    return 1;
  }

  const auto start = rm_nav::common::Now();
  std::size_t packet_count = 0;
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    auto packet = driver.PollPacket();
    if (!packet.has_value()) {
      continue;
    }
    status = writer.WriteImuPacket(*packet);
    if (!status.ok()) {
      rm_nav::utils::LogError("imu_record", status.message);
      writer.Close();
      return 1;
    }
    ++packet_count;
  }

  writer.Close();
  rm_nav::utils::LogInfo("imu_record",
                         "recorded imu packets=" + std::to_string(packet_count) +
                             " file=" + options.output_path);
  return 0;
}
