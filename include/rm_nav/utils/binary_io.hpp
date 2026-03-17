#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_nav::utils {

template <typename T>
void AppendPod(std::vector<std::uint8_t>* buffer, const T& value) {
  const auto* ptr = reinterpret_cast<const std::uint8_t*>(&value);
  buffer->insert(buffer->end(), ptr, ptr + sizeof(T));
}

template <typename T>
bool ReadPod(const std::vector<std::uint8_t>& buffer, std::size_t* offset, T* value) {
  if (*offset + sizeof(T) > buffer.size()) {
    return false;
  }
  std::memcpy(value, buffer.data() + *offset, sizeof(T));
  *offset += sizeof(T);
  return true;
}

}  // namespace rm_nav::utils
