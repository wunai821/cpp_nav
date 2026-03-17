#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/drivers/imu/imu_driver.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  int duration_ms{1000};
  double sample_rate_hz{200.0};
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
    if (arg == "--sample-rate" && index + 1 < argc) {
      options->sample_rate_hz = std::atof(argv[++index]);
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported imu_driver_test arg");
  }
  return rm_nav::common::Status::Ok();
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("imu_test", status.message);
    return 1;
  }

  rm_nav::drivers::imu::ImuDriver driver;
  rm_nav::drivers::imu::ImuDriverConfig config;
  config.sample_rate_hz = options.sample_rate_hz;
  status = driver.Configure(config);
  if (!status.ok()) {
    rm_nav::utils::LogError("imu_test", status.message);
    return 1;
  }

  const auto start = rm_nav::common::Now();
  rm_nav::common::TimePoint last_stamp{};
  std::size_t packet_count = 0;
  double total_dt_ms = 0.0;
  double min_dt_ms = std::numeric_limits<double>::max();
  double max_dt_ms = 0.0;
  double gyro_z_sum = 0.0;
  double accel_z_sum = 0.0;
  double gyro_z_sq_sum = 0.0;
  double accel_z_sq_sum = 0.0;

  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    auto packet = driver.PollPacket();
    if (!packet.has_value()) {
      continue;
    }
    ++packet_count;
    gyro_z_sum += packet->angular_velocity.z;
    accel_z_sum += packet->linear_acceleration.z;
    gyro_z_sq_sum += packet->angular_velocity.z * packet->angular_velocity.z;
    accel_z_sq_sum += packet->linear_acceleration.z * packet->linear_acceleration.z;

    if (packet_count > 1) {
      const double dt_ms =
          static_cast<double>(rm_nav::common::ToNanoseconds(packet->stamp) -
                              rm_nav::common::ToNanoseconds(last_stamp)) /
          1e6;
      total_dt_ms += dt_ms;
      min_dt_ms = std::min(min_dt_ms, dt_ms);
      max_dt_ms = std::max(max_dt_ms, dt_ms);
    }
    last_stamp = packet->stamp;
  }

  const double mean_gyro_z = packet_count == 0 ? 0.0 : gyro_z_sum / packet_count;
  const double mean_accel_z = packet_count == 0 ? 0.0 : accel_z_sum / packet_count;
  const double std_gyro_z =
      packet_count == 0
          ? 0.0
          : std::sqrt(std::max(0.0, gyro_z_sq_sum / packet_count -
                                        mean_gyro_z * mean_gyro_z));
  const double std_accel_z =
      packet_count == 0
          ? 0.0
          : std::sqrt(std::max(0.0, accel_z_sq_sum / packet_count -
                                        mean_accel_z * mean_accel_z));
  const double avg_dt_ms = packet_count <= 1 ? 0.0 : total_dt_ms / (packet_count - 1U);
  const double avg_rate_hz =
      packet_count == 0
          ? 0.0
          : static_cast<double>(packet_count) * 1000.0 / static_cast<double>(options.duration_ms);

  rm_nav::utils::LogInfo(
      "imu_test",
      "packets=" + std::to_string(packet_count) + " rate_hz=" + std::to_string(avg_rate_hz) +
          " dt_ms(avg/min/max)=" + std::to_string(avg_dt_ms) + "/" +
          std::to_string(min_dt_ms == std::numeric_limits<double>::max() ? 0.0 : min_dt_ms) +
          "/" + std::to_string(max_dt_ms) + " gyro_z_mean/std=" +
          std::to_string(mean_gyro_z) + "/" + std::to_string(std_gyro_z) +
          " accel_z_mean/std=" + std::to_string(mean_accel_z) + "/" +
          std::to_string(std_accel_z));
  return 0;
}
