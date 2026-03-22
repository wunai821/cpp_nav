#include "rm_nav/protocol/protocol_codec.hpp"

#include <cstring>

#include "rm_nav/common/time.hpp"
#include "rm_nav/protocol/crc16.hpp"
#include "rm_nav/protocol/crc8.hpp"
#include "rm_nav/utils/binary_io.hpp"

namespace rm_nav::protocol {
namespace {

template <typename T>
void Append(std::vector<std::uint8_t>* payload, const T& value) {
  utils::AppendPod(payload, value);
}

template <typename T>
bool Read(const std::vector<std::uint8_t>& payload, std::size_t* offset, T* value) {
  return utils::ReadPod(payload, offset, value);
}

std::uint16_t ReadU16(const std::vector<std::uint8_t>& buffer, std::size_t offset) {
  return static_cast<std::uint16_t>(buffer[offset]) |
         static_cast<std::uint16_t>(buffer[offset + 1U] << 8U);
}

}  // namespace

std::vector<std::uint8_t> ProtocolCodec::EncodeHeartbeat(
    const HeartbeatPacket& packet) {
  std::vector<std::uint8_t> payload;
  Append(&payload, packet.monotonic_ns);
  Append(&payload, packet.state);
  return EncodePacket(PacketType::kHeartbeat, payload);
}

std::vector<std::uint8_t> ProtocolCodec::EncodeNavCommand(
    const NavCommandPacket& packet) {
  std::vector<std::uint8_t> payload;
  Append(&payload, common::ToNanoseconds(packet.cmd.stamp));
  Append(&payload, packet.cmd.vx_mps);
  Append(&payload, packet.cmd.vy_mps);
  Append(&payload, packet.cmd.wz_radps);
  const std::uint8_t brake = packet.cmd.brake ? 1U : 0U;
  Append(&payload, brake);
  return EncodePacket(PacketType::kNavCommand, payload);
}

std::vector<std::uint8_t> ProtocolCodec::EncodeOdomFeedback(
    const OdomFeedbackPacket& packet) {
  std::vector<std::uint8_t> payload;
  Append(&payload, common::ToNanoseconds(packet.odom.stamp));
  Append(&payload, packet.odom.sequence);
  Append(&payload, packet.odom.x_m);
  Append(&payload, packet.odom.y_m);
  Append(&payload, packet.odom.yaw_rad);
  Append(&payload, packet.odom.vx_mps);
  Append(&payload, packet.odom.vy_mps);
  Append(&payload, packet.odom.wz_radps);
  return EncodePacket(PacketType::kOdomFeedback, payload);
}

std::vector<std::uint8_t> ProtocolCodec::EncodeRefereeState(
    const RefereePacket& packet) {
  std::vector<std::uint8_t> payload;
  Append(&payload, common::ToNanoseconds(packet.state.stamp));
  Append(&payload, packet.state.sequence);
  const std::uint8_t online = packet.state.is_online ? 1U : 0U;
  Append(&payload, online);
  Append(&payload, packet.state.game_stage);
  Append(&payload, packet.state.robot_hp);
  Append(&payload, packet.state.ammo);
  Append(&payload, packet.state.remaining_time_s);
  return EncodePacket(PacketType::kRefereeState, payload);
}

common::Status ProtocolCodec::PushRxBytes(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr && size != 0U) {
    return common::Status::InvalidArgument("rx data pointer is null");
  }
  rx_buffer_.insert(rx_buffer_.end(), data, data + size);
  return common::Status::Ok();
}

std::optional<DecodedPacket> ProtocolCodec::TryDecode() {
  while (rx_buffer_.size() >= kMinimumPacketSize) {
    if (rx_buffer_[0] != kSof0 || rx_buffer_[1] != kSof1) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    if (ComputeCrc8(rx_buffer_.data(), kHeaderSize - 1U) != rx_buffer_[kHeaderSize - 1U]) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    if (rx_buffer_[2] != kProtocolVersion) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const std::uint16_t payload_size = ReadU16(rx_buffer_, 4U);
    if (payload_size > kMaxPayloadSize) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const std::size_t total_size = kHeaderSize + payload_size + kFooterSize;
    if (rx_buffer_.size() < total_size) {
      return std::nullopt;
    }

    const std::uint16_t expected_crc =
        ReadU16(rx_buffer_, kHeaderSize + payload_size);
    const std::uint16_t actual_crc =
        ComputeCrc16(rx_buffer_.data(), kHeaderSize + payload_size);
    if (expected_crc != actual_crc) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    DecodedPacket packet;
    packet.type = static_cast<PacketType>(rx_buffer_[3]);
    packet.sequence = ReadU16(rx_buffer_, 6U);
    packet.payload.assign(rx_buffer_.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
                          rx_buffer_.begin() +
                              static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
    rx_buffer_.erase(rx_buffer_.begin(),
                     rx_buffer_.begin() + static_cast<std::ptrdiff_t>(total_size));
    return packet;
  }
  return std::nullopt;
}

common::Status ProtocolCodec::DecodeHeartbeat(const DecodedPacket& packet,
                                              HeartbeatPacket* heartbeat) const {
  if (heartbeat == nullptr) {
    return common::Status::InvalidArgument("heartbeat output is null");
  }
  if (packet.type != PacketType::kHeartbeat) {
    return common::Status::InvalidArgument("packet type is not heartbeat");
  }
  std::size_t offset = 0;
  if (!Read(packet.payload, &offset, &heartbeat->monotonic_ns) ||
      !Read(packet.payload, &offset, &heartbeat->state)) {
    return common::Status::InvalidArgument("heartbeat payload is truncated");
  }
  return common::Status::Ok();
}

common::Status ProtocolCodec::DecodeNavCommand(const DecodedPacket& packet,
                                               NavCommandPacket* command) const {
  if (command == nullptr) {
    return common::Status::InvalidArgument("command output is null");
  }
  if (packet.type != PacketType::kNavCommand) {
    return common::Status::InvalidArgument("packet type is not nav command");
  }
  std::size_t offset = 0;
  std::int64_t stamp_ns = 0;
  std::uint8_t brake = 0;
  if (!Read(packet.payload, &offset, &stamp_ns) ||
      !Read(packet.payload, &offset, &command->cmd.vx_mps) ||
      !Read(packet.payload, &offset, &command->cmd.vy_mps) ||
      !Read(packet.payload, &offset, &command->cmd.wz_radps) ||
      !Read(packet.payload, &offset, &brake)) {
    return common::Status::InvalidArgument("nav command payload is truncated");
  }
  command->cmd.stamp = common::FromNanoseconds(stamp_ns);
  command->cmd.brake = brake != 0U;
  return common::Status::Ok();
}

common::Status ProtocolCodec::DecodeOdomFeedback(
    const DecodedPacket& packet, OdomFeedbackPacket* odom_feedback) const {
  if (odom_feedback == nullptr) {
    return common::Status::InvalidArgument("odom output is null");
  }
  if (packet.type != PacketType::kOdomFeedback) {
    return common::Status::InvalidArgument("packet type is not odom");
  }
  std::size_t offset = 0;
  std::int64_t stamp_ns = 0;
  if (!Read(packet.payload, &offset, &stamp_ns) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.sequence) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.x_m) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.y_m) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.yaw_rad) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.vx_mps) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.vy_mps) ||
      !Read(packet.payload, &offset, &odom_feedback->odom.wz_radps)) {
    return common::Status::InvalidArgument("odom payload is truncated");
  }
  odom_feedback->odom.stamp = common::FromNanoseconds(stamp_ns);
  return common::Status::Ok();
}

common::Status ProtocolCodec::DecodeRefereeState(const DecodedPacket& packet,
                                                 RefereePacket* referee) const {
  if (referee == nullptr) {
    return common::Status::InvalidArgument("referee output is null");
  }
  if (packet.type != PacketType::kRefereeState) {
    return common::Status::InvalidArgument("packet type is not referee");
  }
  std::size_t offset = 0;
  std::int64_t stamp_ns = 0;
  std::uint8_t online = 0;
  if (!Read(packet.payload, &offset, &stamp_ns) ||
      !Read(packet.payload, &offset, &referee->state.sequence) ||
      !Read(packet.payload, &offset, &online) ||
      !Read(packet.payload, &offset, &referee->state.game_stage) ||
      !Read(packet.payload, &offset, &referee->state.robot_hp) ||
      !Read(packet.payload, &offset, &referee->state.ammo) ||
      !Read(packet.payload, &offset, &referee->state.remaining_time_s)) {
    return common::Status::InvalidArgument("referee payload is truncated");
  }
  referee->state.stamp = common::FromNanoseconds(stamp_ns);
  referee->state.is_online = online != 0U;
  return common::Status::Ok();
}

std::vector<std::uint8_t> ProtocolCodec::EncodePacket(
    PacketType type, const std::vector<std::uint8_t>& payload) {
  if (payload.size() > kMaxPayloadSize) {
    return {};
  }

  std::vector<std::uint8_t> packet;
  packet.reserve(kHeaderSize + payload.size() + kFooterSize);
  packet.push_back(kSof0);
  packet.push_back(kSof1);
  packet.push_back(kProtocolVersion);
  packet.push_back(static_cast<std::uint8_t>(type));
  utils::AppendPod(&packet, static_cast<std::uint16_t>(payload.size()));
  utils::AppendPod(&packet, next_sequence_++);
  packet.push_back(0);
  packet[kHeaderSize - 1U] = ComputeCrc8(packet.data(), kHeaderSize - 1U);
  packet.insert(packet.end(), payload.begin(), payload.end());
  const std::uint16_t crc = ComputeCrc16(packet.data(), packet.size());
  utils::AppendPod(&packet, crc);
  return packet;
}

}  // namespace rm_nav::protocol
