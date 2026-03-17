#pragma once

#include <string>

namespace rm_nav::drivers::serial {

struct SerialConfig {
  std::string device_path{"/dev/ttyUSB0"};
  int baud_rate{115200};
  int read_timeout_ms{20};
  bool non_blocking{true};
};

}  // namespace rm_nav::drivers::serial
