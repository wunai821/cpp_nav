#pragma once

#include <cstddef>
#include <cstdint>

namespace rm_nav::protocol {

std::uint8_t ComputeCrc8(const std::uint8_t* data, std::size_t size);

}  // namespace rm_nav::protocol
