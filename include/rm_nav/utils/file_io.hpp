#pragma once

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/referee_state.hpp"

namespace rm_nav::utils {

enum class RecordChannel : std::uint8_t {
  kLidarFrame = 1,
  kImuPacket = 2,
  kOdomState = 3,
  kRefereeState = 4,
  kChassisCmd = 5,
};

struct RecordedMessage {
  RecordChannel channel{RecordChannel::kLidarFrame};
  std::int64_t stamp_ns{0};
  std::vector<std::uint8_t> payload{};
};

class RecorderWriter {
 public:
  common::Status Open(const std::string& path);
  void Close();

  common::Status WriteLidarFrame(const data::LidarFrame& frame);
  common::Status WriteImuPacket(const data::ImuPacket& packet);
  common::Status WriteOdomState(const data::OdomState& odom);
  common::Status WriteRefereeState(const data::RefereeState& referee);
  common::Status WriteChassisCmd(const data::ChassisCmd& cmd);

 private:
  common::Status WriteRecord(RecordChannel channel, std::int64_t stamp_ns,
                             const std::vector<std::uint8_t>& payload);

  std::ofstream output_{};
};

class RecorderReader {
 public:
  common::Status Open(const std::string& path);
  void Close();
  std::optional<RecordedMessage> ReadNext();

 private:
  std::ifstream input_{};
};

common::Status DecodeLidarFrame(const RecordedMessage& message,
                                data::LidarFrame* frame);
common::Status DecodeImuPacket(const RecordedMessage& message,
                               data::ImuPacket* packet);
common::Status DecodeOdomState(const RecordedMessage& message,
                               data::OdomState* odom);
common::Status DecodeRefereeState(const RecordedMessage& message,
                                  data::RefereeState* referee);
common::Status DecodeChassisCmd(const RecordedMessage& message,
                                data::ChassisCmd* cmd);

}  // namespace rm_nav::utils
