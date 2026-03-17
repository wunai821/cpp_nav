#include "rm_nav/protocol/crc16.hpp"

namespace rm_nav::protocol {

std::uint16_t ComputeCrc16(const std::uint8_t* data, std::size_t size) {
  std::uint16_t crc = 0xFFFF;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

}  // namespace rm_nav::protocol
