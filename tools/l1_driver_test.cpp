#include <algorithm>
#include <cstdlib>
#include <limits>
#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/drivers/lidar/l1_driver.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
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
    return rm_nav::common::Status::InvalidArgument("unsupported l1_driver_test arg");
  }
  return rm_nav::common::Status::Ok();
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_test", status.message);
    return 1;
  }

  rm_nav::drivers::lidar::L1Driver driver;
  rm_nav::drivers::lidar::L1DriverConfig config;
  config.frame_rate_hz = options.frame_rate_hz;
  config.points_per_frame = options.points_per_frame;
  status = driver.Configure(config);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_test", status.message);
    return 1;
  }

  const auto start = rm_nav::common::Now();
  rm_nav::common::TimePoint last_stamp{};
  std::size_t frame_count = 0;
  std::size_t min_points = std::numeric_limits<std::size_t>::max();
  std::size_t max_points = 0;
  double total_dt_ms = 0.0;
  double min_dt_ms = std::numeric_limits<double>::max();
  double max_dt_ms = 0.0;
  bool monotonic = true;

  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    auto frame = driver.PollFrame();
    if (!frame.has_value()) {
      continue;
    }

    ++frame_count;
    min_points = std::min(min_points, frame->points.size());
    max_points = std::max(max_points, frame->points.size());
    if (frame_count > 1) {
      const double dt_ms =
          static_cast<double>(rm_nav::common::ToNanoseconds(frame->stamp) -
                              rm_nav::common::ToNanoseconds(last_stamp)) /
          1e6;
      total_dt_ms += dt_ms;
      min_dt_ms = std::min(min_dt_ms, dt_ms);
      max_dt_ms = std::max(max_dt_ms, dt_ms);
      monotonic = monotonic && frame->stamp >= last_stamp;
    }
    last_stamp = frame->stamp;
  }

  const double avg_fps =
      frame_count == 0
          ? 0.0
          : static_cast<double>(frame_count) * 1000.0 / static_cast<double>(options.duration_ms);
  const double avg_dt_ms = frame_count <= 1 ? 0.0 : total_dt_ms / (frame_count - 1U);
  rm_nav::utils::LogInfo(
      "l1_test",
      "frames=" + std::to_string(frame_count) + " fps=" + std::to_string(avg_fps) +
          " dt_ms(avg/min/max)=" + std::to_string(avg_dt_ms) + "/" +
          std::to_string(min_dt_ms == std::numeric_limits<double>::max() ? 0.0 : min_dt_ms) +
          "/" + std::to_string(max_dt_ms) + " points(min/max)=" +
          std::to_string(min_points == std::numeric_limits<std::size_t>::max() ? 0U : min_points) +
          "/" + std::to_string(max_points) + " monotonic=" + (monotonic ? "true" : "false"));
  return 0;
}
