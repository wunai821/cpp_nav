#include <string>

#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/referee_state.hpp"
#include "rm_nav/utils/file_io.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string path{};
  bool verbose{false};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  if (argc < 2) {
    return rm_nav::common::Status::InvalidArgument("usage: replay_player <record_file> [--verbose]");
  }
  options->path = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--verbose") {
      options->verbose = true;
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported replay_player arg");
  }
  return rm_nav::common::Status::Ok();
}

const char* ChannelToString(rm_nav::utils::RecordChannel channel) {
  switch (channel) {
    case rm_nav::utils::RecordChannel::kLidarFrame:
      return "lidar_frame";
    case rm_nav::utils::RecordChannel::kImuPacket:
      return "imu_packet";
    case rm_nav::utils::RecordChannel::kOdomState:
      return "odom_state";
    case rm_nav::utils::RecordChannel::kRefereeState:
      return "referee_state";
    case rm_nav::utils::RecordChannel::kChassisCmd:
      return "chassis_cmd";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("replay", status.message);
    return 1;
  }

  rm_nav::utils::RecorderReader reader;
  status = reader.Open(options.path);
  if (!status.ok()) {
    rm_nav::utils::LogError("replay", status.message);
    return 1;
  }

  std::size_t total = 0;
  std::size_t lidar_count = 0;
  std::size_t imu_count = 0;
  std::size_t odom_count = 0;
  std::size_t referee_count = 0;
  std::size_t cmd_count = 0;

  while (true) {
    auto message = reader.ReadNext();
    if (!message.has_value()) {
      break;
    }
    ++total;
    switch (message->channel) {
      case rm_nav::utils::RecordChannel::kLidarFrame: {
        ++lidar_count;
        if (options.verbose) {
          rm_nav::data::LidarFrame frame;
          rm_nav::utils::DecodeLidarFrame(*message, &frame);
          rm_nav::utils::LogInfo("replay",
                                 std::string(ChannelToString(message->channel)) +
                                     " frame_index=" + std::to_string(frame.frame_index) +
                                     " points=" + std::to_string(frame.points.size()));
        }
        break;
      }
      case rm_nav::utils::RecordChannel::kImuPacket: {
        ++imu_count;
        if (options.verbose) {
          rm_nav::data::ImuPacket packet;
          rm_nav::utils::DecodeImuPacket(*message, &packet);
          rm_nav::utils::LogInfo("replay",
                                 std::string(ChannelToString(message->channel)) +
                                     " sample_index=" + std::to_string(packet.sample_index));
        }
        break;
      }
      case rm_nav::utils::RecordChannel::kOdomState:
        ++odom_count;
        break;
      case rm_nav::utils::RecordChannel::kRefereeState:
        ++referee_count;
        break;
      case rm_nav::utils::RecordChannel::kChassisCmd:
        ++cmd_count;
        break;
    }
  }

  rm_nav::utils::LogInfo("replay",
                         "summary total=" + std::to_string(total) + " lidar=" +
                             std::to_string(lidar_count) + " imu=" +
                             std::to_string(imu_count) + " odom=" +
                             std::to_string(odom_count) + " referee=" +
                             std::to_string(referee_count) + " cmd=" +
                             std::to_string(cmd_count));
  return 0;
}
