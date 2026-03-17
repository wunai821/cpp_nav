#include "rm_nav/utils/file_io.hpp"

#include <array>
#include <filesystem>

#include "rm_nav/common/time.hpp"
#include "rm_nav/utils/binary_io.hpp"

namespace rm_nav::utils {
namespace {

inline constexpr std::array<char, 8> kRecorderMagic = {'R', 'M', 'R', 'E', 'C', '0', '1', '\n'};
inline constexpr std::uint32_t kRecorderVersion = 1;

std::vector<std::uint8_t> EncodeLidarPayload(const data::LidarFrame& frame) {
  std::vector<std::uint8_t> payload;
  AppendPod(&payload, common::ToNanoseconds(frame.scan_begin_stamp));
  AppendPod(&payload, common::ToNanoseconds(frame.scan_end_stamp));
  AppendPod(&payload, frame.frame_index);
  AppendPod(&payload, static_cast<std::uint32_t>(frame.points.size()));
  const std::uint8_t deskewed = frame.is_deskewed ? 1U : 0U;
  AppendPod(&payload, deskewed);
  for (const auto& point : frame.points) {
    AppendPod(&payload, point.x);
    AppendPod(&payload, point.y);
    AppendPod(&payload, point.z);
    AppendPod(&payload, point.intensity);
    AppendPod(&payload, point.relative_time_s);
  }
  return payload;
}

std::vector<std::uint8_t> EncodeImuPayload(const data::ImuPacket& packet) {
  std::vector<std::uint8_t> payload;
  AppendPod(&payload, packet.sample_index);
  AppendPod(&payload, packet.angular_velocity.x);
  AppendPod(&payload, packet.angular_velocity.y);
  AppendPod(&payload, packet.angular_velocity.z);
  AppendPod(&payload, packet.linear_acceleration.x);
  AppendPod(&payload, packet.linear_acceleration.y);
  AppendPod(&payload, packet.linear_acceleration.z);
  const std::uint8_t is_valid = packet.is_valid ? 1U : 0U;
  AppendPod(&payload, is_valid);
  return payload;
}

std::vector<std::uint8_t> EncodeOdomPayload(const data::OdomState& odom) {
  std::vector<std::uint8_t> payload;
  AppendPod(&payload, odom.sequence);
  AppendPod(&payload, odom.x_m);
  AppendPod(&payload, odom.y_m);
  AppendPod(&payload, odom.yaw_rad);
  AppendPod(&payload, odom.vx_mps);
  AppendPod(&payload, odom.vy_mps);
  AppendPod(&payload, odom.wz_radps);
  return payload;
}

std::vector<std::uint8_t> EncodeRefereePayload(const data::RefereeState& referee) {
  std::vector<std::uint8_t> payload;
  AppendPod(&payload, referee.sequence);
  const std::uint8_t online = referee.is_online ? 1U : 0U;
  AppendPod(&payload, online);
  AppendPod(&payload, referee.game_stage);
  AppendPod(&payload, referee.robot_hp);
  AppendPod(&payload, referee.ammo);
  AppendPod(&payload, referee.remaining_time_s);
  return payload;
}

std::vector<std::uint8_t> EncodeCmdPayload(const data::ChassisCmd& cmd) {
  std::vector<std::uint8_t> payload;
  AppendPod(&payload, cmd.vx_mps);
  AppendPod(&payload, cmd.vy_mps);
  AppendPod(&payload, cmd.wz_radps);
  const std::uint8_t brake = cmd.brake ? 1U : 0U;
  AppendPod(&payload, brake);
  return payload;
}

common::Status DecodeLidarPayload(const std::vector<std::uint8_t>& payload,
                                  data::LidarFrame* frame) {
  std::size_t offset = 0;
  common::TimeNs scan_begin_ns = 0;
  common::TimeNs scan_end_ns = 0;
  std::uint32_t point_count = 0;
  std::uint8_t deskewed = 0;
  if (!ReadPod(payload, &offset, &scan_begin_ns) ||
      !ReadPod(payload, &offset, &scan_end_ns) ||
      !ReadPod(payload, &offset, &frame->frame_index) ||
      !ReadPod(payload, &offset, &point_count) ||
      !ReadPod(payload, &offset, &deskewed)) {
    return common::Status::InvalidArgument("lidar record payload is truncated");
  }
  frame->scan_begin_stamp = common::FromNanoseconds(scan_begin_ns);
  frame->scan_end_stamp = common::FromNanoseconds(scan_end_ns);
  frame->stamp = frame->scan_end_stamp;
  frame->points.clear();
  frame->points.reserve(point_count);
  frame->is_deskewed = deskewed != 0U;
  for (std::uint32_t index = 0; index < point_count; ++index) {
    data::PointXYZI point;
    if (!ReadPod(payload, &offset, &point.x) || !ReadPod(payload, &offset, &point.y) ||
        !ReadPod(payload, &offset, &point.z) ||
        !ReadPod(payload, &offset, &point.intensity) ||
        !ReadPod(payload, &offset, &point.relative_time_s)) {
      return common::Status::InvalidArgument("lidar point payload is truncated");
    }
    frame->points.push_back(point);
  }
  return common::Status::Ok();
}

}  // namespace

common::Status RecorderWriter::Open(const std::string& path) {
  Close();
  const std::filesystem::path file_path(path);
  if (file_path.has_parent_path()) {
    std::filesystem::create_directories(file_path.parent_path());
  }
  output_.open(path, std::ios::binary);
  if (!output_.is_open()) {
    return common::Status::Unavailable("failed to open recorder output");
  }
  output_.write(kRecorderMagic.data(), static_cast<std::streamsize>(kRecorderMagic.size()));
  output_.write(reinterpret_cast<const char*>(&kRecorderVersion),
                static_cast<std::streamsize>(sizeof(kRecorderVersion)));
  return common::Status::Ok();
}

void RecorderWriter::Close() {
  if (output_.is_open()) {
    output_.close();
  }
}

common::Status RecorderWriter::WriteLidarFrame(const data::LidarFrame& frame) {
  return WriteRecord(RecordChannel::kLidarFrame, common::ToNanoseconds(frame.stamp),
                     EncodeLidarPayload(frame));
}

common::Status RecorderWriter::WriteImuPacket(const data::ImuPacket& packet) {
  return WriteRecord(RecordChannel::kImuPacket, common::ToNanoseconds(packet.stamp),
                     EncodeImuPayload(packet));
}

common::Status RecorderWriter::WriteOdomState(const data::OdomState& odom) {
  return WriteRecord(RecordChannel::kOdomState, common::ToNanoseconds(odom.stamp),
                     EncodeOdomPayload(odom));
}

common::Status RecorderWriter::WriteRefereeState(const data::RefereeState& referee) {
  return WriteRecord(RecordChannel::kRefereeState, common::ToNanoseconds(referee.stamp),
                     EncodeRefereePayload(referee));
}

common::Status RecorderWriter::WriteChassisCmd(const data::ChassisCmd& cmd) {
  return WriteRecord(RecordChannel::kChassisCmd, common::ToNanoseconds(cmd.stamp),
                     EncodeCmdPayload(cmd));
}

common::Status RecorderWriter::WriteRecord(RecordChannel channel, std::int64_t stamp_ns,
                                           const std::vector<std::uint8_t>& payload) {
  if (!output_.is_open()) {
    return common::Status::NotReady("recorder output is not open");
  }
  const auto channel_value = static_cast<std::uint8_t>(channel);
  const auto payload_size = static_cast<std::uint32_t>(payload.size());
  output_.write(reinterpret_cast<const char*>(&channel_value), sizeof(channel_value));
  output_.write(reinterpret_cast<const char*>(&stamp_ns), sizeof(stamp_ns));
  output_.write(reinterpret_cast<const char*>(&payload_size), sizeof(payload_size));
  output_.write(reinterpret_cast<const char*>(payload.data()),
                static_cast<std::streamsize>(payload.size()));
  return output_.good() ? common::Status::Ok()
                        : common::Status::InternalError("failed to write record");
}

common::Status RecorderReader::Open(const std::string& path) {
  Close();
  input_.open(path, std::ios::binary);
  if (!input_.is_open()) {
    return common::Status::Unavailable("failed to open recorder input");
  }
  std::array<char, 8> magic {};
  std::uint32_t version = 0;
  input_.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  input_.read(reinterpret_cast<char*>(&version), static_cast<std::streamsize>(sizeof(version)));
  if (!input_.good() || magic != kRecorderMagic || version != kRecorderVersion) {
    Close();
    return common::Status::InvalidArgument("invalid recorder file header");
  }
  return common::Status::Ok();
}

void RecorderReader::Close() {
  if (input_.is_open()) {
    input_.close();
  }
}

std::optional<RecordedMessage> RecorderReader::ReadNext() {
  if (!input_.is_open() || input_.peek() == std::ifstream::traits_type::eof()) {
    return std::nullopt;
  }

  RecordedMessage message;
  std::uint8_t channel = 0;
  std::uint32_t payload_size = 0;
  input_.read(reinterpret_cast<char*>(&channel), sizeof(channel));
  input_.read(reinterpret_cast<char*>(&message.stamp_ns), sizeof(message.stamp_ns));
  input_.read(reinterpret_cast<char*>(&payload_size), sizeof(payload_size));
  if (!input_.good()) {
    return std::nullopt;
  }
  message.channel = static_cast<RecordChannel>(channel);
  message.payload.resize(payload_size);
  input_.read(reinterpret_cast<char*>(message.payload.data()),
              static_cast<std::streamsize>(payload_size));
  if (!input_.good()) {
    return std::nullopt;
  }
  return message;
}

common::Status DecodeLidarFrame(const RecordedMessage& message, data::LidarFrame* frame) {
  if (frame == nullptr || message.channel != RecordChannel::kLidarFrame) {
    return common::Status::InvalidArgument("record is not a lidar frame");
  }
  frame->stamp = common::FromNanoseconds(message.stamp_ns);
  return DecodeLidarPayload(message.payload, frame);
}

common::Status DecodeImuPacket(const RecordedMessage& message, data::ImuPacket* packet) {
  if (packet == nullptr || message.channel != RecordChannel::kImuPacket) {
    return common::Status::InvalidArgument("record is not an imu packet");
  }
  packet->stamp = common::FromNanoseconds(message.stamp_ns);
  std::size_t offset = 0;
  std::uint8_t is_valid = 0;
  if (!ReadPod(message.payload, &offset, &packet->sample_index) ||
      !ReadPod(message.payload, &offset, &packet->angular_velocity.x) ||
      !ReadPod(message.payload, &offset, &packet->angular_velocity.y) ||
      !ReadPod(message.payload, &offset, &packet->angular_velocity.z) ||
      !ReadPod(message.payload, &offset, &packet->linear_acceleration.x) ||
      !ReadPod(message.payload, &offset, &packet->linear_acceleration.y) ||
      !ReadPod(message.payload, &offset, &packet->linear_acceleration.z) ||
      !ReadPod(message.payload, &offset, &is_valid)) {
    return common::Status::InvalidArgument("imu record payload is truncated");
  }
  packet->is_valid = is_valid != 0U;
  return common::Status::Ok();
}

common::Status DecodeOdomState(const RecordedMessage& message, data::OdomState* odom) {
  if (odom == nullptr || message.channel != RecordChannel::kOdomState) {
    return common::Status::InvalidArgument("record is not an odom state");
  }
  odom->stamp = common::FromNanoseconds(message.stamp_ns);
  std::size_t offset = 0;
  if (!ReadPod(message.payload, &offset, &odom->sequence) ||
      !ReadPod(message.payload, &offset, &odom->x_m) ||
      !ReadPod(message.payload, &offset, &odom->y_m) ||
      !ReadPod(message.payload, &offset, &odom->yaw_rad) ||
      !ReadPod(message.payload, &offset, &odom->vx_mps) ||
      !ReadPod(message.payload, &offset, &odom->vy_mps) ||
      !ReadPod(message.payload, &offset, &odom->wz_radps)) {
    return common::Status::InvalidArgument("odom record payload is truncated");
  }
  return common::Status::Ok();
}

common::Status DecodeRefereeState(const RecordedMessage& message,
                                  data::RefereeState* referee) {
  if (referee == nullptr || message.channel != RecordChannel::kRefereeState) {
    return common::Status::InvalidArgument("record is not a referee state");
  }
  referee->stamp = common::FromNanoseconds(message.stamp_ns);
  std::size_t offset = 0;
  std::uint8_t online = 0;
  if (!ReadPod(message.payload, &offset, &referee->sequence) ||
      !ReadPod(message.payload, &offset, &online) ||
      !ReadPod(message.payload, &offset, &referee->game_stage) ||
      !ReadPod(message.payload, &offset, &referee->robot_hp) ||
      !ReadPod(message.payload, &offset, &referee->ammo) ||
      !ReadPod(message.payload, &offset, &referee->remaining_time_s)) {
    return common::Status::InvalidArgument("referee record payload is truncated");
  }
  referee->is_online = online != 0U;
  return common::Status::Ok();
}

common::Status DecodeChassisCmd(const RecordedMessage& message, data::ChassisCmd* cmd) {
  if (cmd == nullptr || message.channel != RecordChannel::kChassisCmd) {
    return common::Status::InvalidArgument("record is not a chassis cmd");
  }
  cmd->stamp = common::FromNanoseconds(message.stamp_ns);
  std::size_t offset = 0;
  std::uint8_t brake = 0;
  if (!ReadPod(message.payload, &offset, &cmd->vx_mps) ||
      !ReadPod(message.payload, &offset, &cmd->vy_mps) ||
      !ReadPod(message.payload, &offset, &cmd->wz_radps) ||
      !ReadPod(message.payload, &offset, &brake)) {
    return common::Status::InvalidArgument("cmd record payload is truncated");
  }
  cmd->brake = brake != 0U;
  return common::Status::Ok();
}

}  // namespace rm_nav::utils
