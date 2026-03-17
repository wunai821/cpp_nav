#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/protocol/heartbeat_packet.hpp"
#include "rm_nav/protocol/nav_command_packet.hpp"
#include "rm_nav/protocol/odom_feedback_packet.hpp"
#include "rm_nav/protocol/packet_layout.hpp"
#include "rm_nav/protocol/referee_packet.hpp"

namespace rm_nav::protocol {

struct DecodedPacket {
  PacketType type{PacketType::kHeartbeat};
  std::uint16_t sequence{0};
  std::vector<std::uint8_t> payload{};
};

class ProtocolCodec {
 public:
  std::vector<std::uint8_t> EncodeHeartbeat(const HeartbeatPacket& packet);
  std::vector<std::uint8_t> EncodeNavCommand(const NavCommandPacket& packet);
  std::vector<std::uint8_t> EncodeOdomFeedback(const OdomFeedbackPacket& packet);
  std::vector<std::uint8_t> EncodeRefereeState(const RefereePacket& packet);

  common::Status PushRxBytes(const std::uint8_t* data, std::size_t size);
  std::optional<DecodedPacket> TryDecode();

  common::Status DecodeHeartbeat(const DecodedPacket& packet,
                                 HeartbeatPacket* heartbeat) const;
  common::Status DecodeNavCommand(const DecodedPacket& packet,
                                  NavCommandPacket* command) const;
  common::Status DecodeOdomFeedback(const DecodedPacket& packet,
                                    OdomFeedbackPacket* odom_feedback) const;
  common::Status DecodeRefereeState(const DecodedPacket& packet,
                                    RefereePacket* referee) const;

 private:
  std::vector<std::uint8_t> EncodePacket(PacketType type,
                                         const std::vector<std::uint8_t>& payload);

  std::vector<std::uint8_t> rx_buffer_{};
  std::uint16_t next_sequence_{0};
};

}  // namespace rm_nav::protocol
