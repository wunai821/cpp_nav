#pragma once

#include <cstddef>
#include <cstdint>

#include "rm_nav/common/status.hpp"
#include "rm_nav/drivers/serial/serial_config.hpp"

namespace rm_nav::drivers::serial {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  common::Status Open(const SerialConfig& config);
  void Close();
  bool IsOpen() const;
  int fd() const { return fd_; }

  common::Status Write(const std::uint8_t* data, std::size_t size,
                       std::size_t* bytes_written = nullptr);
  common::Status Read(std::uint8_t* buffer, std::size_t capacity,
                      std::size_t* bytes_read = nullptr);

 private:
  int fd_{-1};
  SerialConfig config_{};
};

}  // namespace rm_nav::drivers::serial
