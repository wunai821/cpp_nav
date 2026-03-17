#pragma once

#include <cstddef>
#include <cstdint>

namespace rm_nav::protocol {

inline constexpr std::uint8_t kSof0 = 0xA5;
inline constexpr std::uint8_t kSof1 = 0x5A;
inline constexpr std::uint8_t kProtocolVersion = 1;
inline constexpr std::size_t kHeaderSize = 9;
inline constexpr std::size_t kFooterSize = 2;
inline constexpr std::size_t kMinimumPacketSize = kHeaderSize + kFooterSize;
inline constexpr std::size_t kMaxPayloadSize = 1024 * 64;

enum class PacketType : std::uint8_t {
  kHeartbeat = 1,
  kNavCommand = 2,
  kOdomFeedback = 3,
  kRefereeState = 4,
};

struct PacketHeader {
  std::uint8_t sof0{kSof0};
  std::uint8_t sof1{kSof1};
  std::uint8_t version{kProtocolVersion};
  PacketType type{PacketType::kHeartbeat};
  std::uint16_t payload_size{0};
  std::uint16_t sequence{0};
  std::uint8_t header_crc8{0};
};

}  // namespace rm_nav::protocol
