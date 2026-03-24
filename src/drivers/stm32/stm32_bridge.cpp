#include "rm_nav/drivers/stm32/stm32_bridge.hpp"

namespace rm_nav::drivers::stm32 {

Stm32Bridge::~Stm32Bridge() { Close(); }

common::Status Stm32Bridge::Configure(const Stm32BridgeConfig& config) {
  Close();
  config_ = config;
  health_report_ = {};
  latest_odom_.reset();
  latest_referee_.reset();
  return serial_port_.Open(config.serial);
}

void Stm32Bridge::Close() { serial_port_.Close(); }

bool Stm32Bridge::IsOpen() const { return serial_port_.IsOpen(); }

common::Status Stm32Bridge::SendHeartbeat() {
  if (!IsOpen()) {
    return common::Status::NotReady("stm32 bridge is not open");
  }
  protocol::HeartbeatPacket heartbeat;
  heartbeat.monotonic_ns = static_cast<std::uint64_t>(common::ToNanoseconds(common::Now()));
  heartbeat.state = config_.heartbeat_state;
  const auto bytes = codec_.EncodeHeartbeat(heartbeat);
  std::size_t written = 0;
  auto status = serial_port_.Write(bytes.data(), bytes.size(), &written);
  if (!status.ok()) {
    return status;
  }
  if (written != bytes.size()) {
    ++health_report_.dropped_packets;
    return common::Status::InternalError("partial heartbeat write");
  }
  ++health_report_.tx_packets;
  return common::Status::Ok();
}

common::Status Stm32Bridge::SendChassisCmd(const data::ChassisCmd& cmd) {
  if (!IsOpen()) {
    return common::Status::NotReady("stm32 bridge is not open");
  }
  protocol::NavCommandPacket packet;
  packet.cmd = cmd;
  const auto bytes = codec_.EncodeNavCommand(packet);
  std::size_t written = 0;
  auto status = serial_port_.Write(bytes.data(), bytes.size(), &written);
  if (!status.ok()) {
    return status;
  }
  if (written != bytes.size()) {
    ++health_report_.dropped_packets;
    return common::Status::InternalError("partial cmd write");
  }
  ++health_report_.tx_packets;
  return common::Status::Ok();
}

bool Stm32Bridge::WaitForRx(std::chrono::milliseconds timeout) const {
  if (!IsOpen()) {
    return false;
  }
  return serial_port_.WaitReadable(timeout);
}

common::Status Stm32Bridge::SpinOnce() {
  if (!IsOpen()) {
    return common::Status::NotReady("stm32 bridge is not open");
  }

  std::uint8_t buffer[512];
  std::size_t bytes_read = 0;
  auto status = serial_port_.Read(buffer, sizeof(buffer), &bytes_read);
  if (!status.ok()) {
    return status;
  }
  if (bytes_read != 0U) {
    status = codec_.PushRxBytes(buffer, bytes_read);
    if (!status.ok()) {
      return status;
    }
  }

  while (true) {
    auto decoded = codec_.TryDecode();
    if (!decoded.has_value()) {
      break;
    }

    ++health_report_.rx_packets;
    if (decoded->type == protocol::PacketType::kOdomFeedback) {
      protocol::OdomFeedbackPacket packet;
      status = codec_.DecodeOdomFeedback(*decoded, &packet);
      if (!status.ok()) {
        ++health_report_.parse_errors;
        continue;
      }
      latest_odom_ = packet.odom;
      continue;
    }
    if (decoded->type == protocol::PacketType::kRefereeState) {
      protocol::RefereePacket packet;
      status = codec_.DecodeRefereeState(*decoded, &packet);
      if (!status.ok()) {
        ++health_report_.parse_errors;
        continue;
      }
      latest_referee_ = packet.state;
      continue;
    }
  }

  return common::Status::Ok();
}

std::optional<data::OdomState> Stm32Bridge::TakeOdomState() {
  auto value = latest_odom_;
  latest_odom_.reset();
  return value;
}

std::optional<data::RefereeState> Stm32Bridge::TakeRefereeState() {
  auto value = latest_referee_;
  latest_referee_.reset();
  return value;
}

}  // namespace rm_nav::drivers::stm32
