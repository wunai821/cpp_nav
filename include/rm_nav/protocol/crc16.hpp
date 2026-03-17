#pragma once

#include <cstddef>
#include <cstdint>

namespace rm_nav::protocol {

std::uint16_t ComputeCrc16(const std::uint8_t* data, std::size_t size);

}  // namespace rm_nav::protocol
