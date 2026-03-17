#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/drivers/lidar/l1_decoder.hpp"
#include "rm_nav/drivers/lidar/l1_driver.hpp"
#include "rm_nav/drivers/lidar/l1_packet_parser.hpp"
#include "rm_nav/drivers/serial/serial_port.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string source{"synthetic"};
  std::string device{};
  int baud_rate{2000000};
  int cloud_scan_num{18};
  int read_timeout_ms{20};
  int duration_ms{1000};
  double frame_rate_hz{10.0};
  std::size_t points_per_frame{360};
  std::size_t max_dump_bytes{24};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--source" && index + 1 < argc) {
      options->source = argv[++index];
      continue;
    }
    if (arg == "--device" && index + 1 < argc) {
      options->device = argv[++index];
      continue;
    }
    if (arg == "--baud-rate" && index + 1 < argc) {
      options->baud_rate = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--cloud-scan-num" && index + 1 < argc) {
      options->cloud_scan_num = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--read-timeout-ms" && index + 1 < argc) {
      options->read_timeout_ms = std::atoi(argv[++index]);
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
    if (arg == "--max-dump-bytes" && index + 1 < argc) {
      options->max_dump_bytes = static_cast<std::size_t>(std::atoi(argv[++index]));
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported l1_driver_test arg");
  }
  return rm_nav::common::Status::Ok();
}

std::string HexDump(const std::vector<std::uint8_t>& bytes, std::size_t max_dump_bytes) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  const std::size_t count = std::min(bytes.size(), max_dump_bytes);
  for (std::size_t index = 0; index < count; ++index) {
    if (index != 0U) {
      output << ' ';
    }
    output << std::setw(2) << static_cast<int>(bytes[index]);
  }
  if (bytes.size() > count) {
    output << " ...";
  }
  return output.str();
}

int RunSynthetic(const Options& options) {
  rm_nav::drivers::lidar::L1Driver driver;
  rm_nav::drivers::lidar::L1DriverConfig config;
  config.source = "synthetic";
  config.frame_rate_hz = options.frame_rate_hz;
  config.points_per_frame = options.points_per_frame;
  auto status = driver.Configure(config);
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
      "synthetic frames=" + std::to_string(frame_count) + " fps=" + std::to_string(avg_fps) +
          " dt_ms(avg/min/max)=" + std::to_string(avg_dt_ms) + "/" +
          std::to_string(min_dt_ms == std::numeric_limits<double>::max() ? 0.0 : min_dt_ms) +
          "/" + std::to_string(max_dt_ms) + " points(min/max)=" +
          std::to_string(min_points == std::numeric_limits<std::size_t>::max() ? 0U : min_points) +
          "/" + std::to_string(max_points) + " monotonic=" + (monotonic ? "true" : "false"));
  return 0;
}

int RunUnitreeSdk(const Options& options) {
  if (options.device.empty()) {
    rm_nav::utils::LogError("l1_test", "unitree_sdk mode requires --device <path>");
    return 1;
  }

  rm_nav::drivers::lidar::L1Driver driver;
  rm_nav::drivers::lidar::L1DriverConfig config;
  config.source = "unitree_sdk";
  config.device_path = options.device;
  config.baud_rate = options.baud_rate;
  config.cloud_scan_num = options.cloud_scan_num;
  const auto status = driver.Configure(config);
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
      "unitree_sdk device=" + options.device + " baud=" + std::to_string(options.baud_rate) +
          " cloud_scan_num=" + std::to_string(options.cloud_scan_num) +
          " frames=" + std::to_string(frame_count) + " fps=" + std::to_string(avg_fps) +
          " dt_ms(avg/min/max)=" + std::to_string(avg_dt_ms) + "/" +
          std::to_string(min_dt_ms == std::numeric_limits<double>::max() ? 0.0 : min_dt_ms) +
          "/" + std::to_string(max_dt_ms) + " points(min/max)=" +
          std::to_string(min_points == std::numeric_limits<std::size_t>::max() ? 0U : min_points) +
          "/" + std::to_string(max_points) + " monotonic=" + (monotonic ? "true" : "false"));
  return frame_count == 0U ? 2 : 0;
}

int RunSerial(const Options& options) {
  if (options.device.empty()) {
    rm_nav::utils::LogError("l1_test", "serial mode requires --device <path>");
    return 1;
  }

  rm_nav::drivers::serial::SerialPort serial;
  rm_nav::drivers::serial::SerialConfig serial_config;
  serial_config.device_path = options.device;
  serial_config.baud_rate = options.baud_rate;
  serial_config.read_timeout_ms = options.read_timeout_ms;
  const auto status = serial.Open(serial_config);
  if (!status.ok()) {
    rm_nav::utils::LogError("l1_test", status.message + ": " + options.device);
    return 1;
  }

  rm_nav::drivers::lidar::L1PacketParser parser;
  rm_nav::drivers::lidar::L1Decoder decoder;
  std::array<std::uint8_t, 4096> buffer{};
  const auto start = rm_nav::common::Now();
  rm_nav::common::TimePoint last_data_stamp{};
  std::size_t read_count = 0;
  std::size_t packet_count = 0;
  std::size_t decoded_frames = 0;
  std::size_t decode_unimplemented = 0;
  std::size_t total_bytes = 0;
  std::size_t min_bytes = std::numeric_limits<std::size_t>::max();
  std::size_t max_bytes = 0;
  double total_dt_ms = 0.0;
  double min_dt_ms = std::numeric_limits<double>::max();
  double max_dt_ms = 0.0;
  bool monotonic = true;
  std::string first_packet_hex{};

  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    std::size_t bytes_read = 0;
    const auto read_status = serial.Read(buffer.data(), buffer.size(), &bytes_read);
    if (!read_status.ok()) {
      rm_nav::utils::LogError("l1_test", read_status.message);
      return 1;
    }
    if (bytes_read == 0U) {
      continue;
    }

    ++read_count;
    ++packet_count;
    total_bytes += bytes_read;
    min_bytes = std::min(min_bytes, bytes_read);
    max_bytes = std::max(max_bytes, bytes_read);
    const auto now = rm_nav::common::Now();
    if (read_count > 1) {
      const double dt_ms =
          static_cast<double>(rm_nav::common::ToNanoseconds(now) -
                              rm_nav::common::ToNanoseconds(last_data_stamp)) /
          1e6;
      total_dt_ms += dt_ms;
      min_dt_ms = std::min(min_dt_ms, dt_ms);
      max_dt_ms = std::max(max_dt_ms, dt_ms);
      monotonic = monotonic && now >= last_data_stamp;
    }
    last_data_stamp = now;

    rm_nav::drivers::lidar::L1Packet packet;
    const auto parse_status = parser.Parse(buffer.data(), bytes_read, &packet);
    if (!parse_status.ok()) {
      rm_nav::utils::LogWarn("l1_test", "packet parse failed: " + parse_status.message);
      continue;
    }
    if (first_packet_hex.empty()) {
      first_packet_hex = HexDump(packet.bytes, options.max_dump_bytes);
    }

    rm_nav::data::LidarFrame frame;
    const auto decode_status = decoder.Decode(packet, &frame);
    if (decode_status.ok()) {
      ++decoded_frames;
    } else if (decode_status.code == rm_nav::common::StatusCode::kUnimplemented) {
      ++decode_unimplemented;
    }
  }

  serial.Close();

  const double avg_packet_hz =
      packet_count == 0
          ? 0.0
          : static_cast<double>(packet_count) * 1000.0 /
                static_cast<double>(options.duration_ms);
  const double avg_bytes = packet_count == 0
                               ? 0.0
                               : static_cast<double>(total_bytes) /
                                     static_cast<double>(packet_count);
  const double avg_dt_ms = packet_count <= 1 ? 0.0 : total_dt_ms / (packet_count - 1U);
  rm_nav::utils::LogInfo(
      "l1_test",
      "serial device=" + options.device + " baud=" + std::to_string(options.baud_rate) +
          " reads=" + std::to_string(read_count) + " packets=" +
          std::to_string(packet_count) + " packet_hz=" + std::to_string(avg_packet_hz) +
          " bytes(total/avg/min/max)=" + std::to_string(total_bytes) + "/" +
          std::to_string(avg_bytes) + "/" +
          std::to_string(min_bytes == std::numeric_limits<std::size_t>::max() ? 0U : min_bytes) +
          "/" + std::to_string(max_bytes) + " dt_ms(avg/min/max)=" +
          std::to_string(avg_dt_ms) + "/" +
          std::to_string(min_dt_ms == std::numeric_limits<double>::max() ? 0.0 : min_dt_ms) +
          "/" + std::to_string(max_dt_ms) + " monotonic=" + (monotonic ? "true" : "false") +
          " decoded_frames=" + std::to_string(decoded_frames) +
          " decode_unimplemented=" + std::to_string(decode_unimplemented));
  if (!first_packet_hex.empty()) {
    rm_nav::utils::LogInfo("l1_test", "first_packet_hex=" + first_packet_hex);
  } else {
    rm_nav::utils::LogWarn("l1_test", "no serial data received from device");
  }
  if (decode_unimplemented > 0U) {
    rm_nav::utils::LogWarn("l1_test",
                           "raw packet bring-up is working, but hardware L1 decode is not implemented yet");
  }
  return packet_count == 0U ? 2 : 0;
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

  if (options.source == "synthetic") {
    return RunSynthetic(options);
  }
  if (options.source == "serial") {
    return RunSerial(options);
  }
  if (options.source == "unitree_sdk") {
    return RunUnitreeSdk(options);
  }

  rm_nav::utils::LogError("l1_test", "unsupported source: " + options.source);
  return 1;
}
