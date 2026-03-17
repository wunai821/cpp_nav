#include "rm_nav/protocol/crc8.hpp"

namespace rm_nav::protocol {

std::uint8_t ComputeCrc8(const std::uint8_t* data, std::size_t size) {
  std::uint8_t crc = 0xFF;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ? static_cast<std::uint8_t>((crc << 1U) ^ 0x31U)
                                : static_cast<std::uint8_t>(crc << 1U);
    }
  }
  return crc;
}

}  // namespace rm_nav::protocol
